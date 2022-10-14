use core::task::Poll;

use cortex_m_interrupt::IrqHandle;
pub use cortex_m_interrupt::{WakerConsumer, WakerProducer, WakerQueue};
use embedded_dma::ReadBuffer;

use crate::{pac, spi::Tx};

type DMA1Stream4 = Stream4<pac::DMA1>;

use super::{
    traits::{Channel, DMASet, PeriAddress, Stream},
    ChannelX, DMAError, MemoryToPeripheral, Stream4, Transfer,
};

pub type NormalSpiTransfer<'buf> =
    Transfer<Stream4<pac::DMA1>, 0, Tx<pac::SPI2>, MemoryToPeripheral, &'buf mut [u8; 128]>;

pub type SpiTransfer<'waker, 'buf> = AsyncTransfer<
    'waker,
    Stream4<pac::DMA1>,
    0,
    Tx<pac::SPI2>,
    MemoryToPeripheral,
    &'buf mut [u8; 128],
>;

pub struct AsyncTransferStorage {
    waker_queue: WakerQueue,
}

impl AsyncTransferStorage {
    pub fn new() -> Self {
        Self {
            waker_queue: WakerQueue::new(),
        }
    }

    pub fn split<'buf, T: IrqHandle>(
        &'static mut self,
        handle: T,
        transfer: NormalSpiTransfer<'buf>,
    ) -> SpiTransfer<'static, 'buf> {
        use core::mem::MaybeUninit;
        static mut WAKER: MaybeUninit<WakerConsumer<'static>> = MaybeUninit::uninit();

        let (r, w) = self.waker_queue.split();

        unsafe { WAKER = MaybeUninit::new(r) };

        handle.register(|| {
            cortex_m::interrupt::free(|_| {
                unsafe { DMA1Stream4::st() }.cr.modify(|_, w| {
                    w.tcie()
                        .bit(false)
                        .htie()
                        .bit(false)
                        .teie()
                        .bit(false)
                        .dmeie()
                        .bit(false)
                });
            });

            if let Some(waker) = unsafe { WAKER.assume_init_mut().dequeue() } {
                waker.wake();
            }
        });

        AsyncTransfer::new(w, transfer)
    }
}

pub struct AsyncTransfer<'waker, STREAM, const CHANNEL: u8, PERIPHERAL, DIRECTION, BUF>
where
    STREAM: Stream,
    PERIPHERAL: PeriAddress,
{
    inner: Transfer<STREAM, CHANNEL, PERIPHERAL, DIRECTION, BUF>,
    waker_producer: WakerProducer<'waker>,
}

impl<'waker, STREAM, const CHANNEL: u8, PERIPHERAL, DIRECTION, BUF>
    AsyncTransfer<'waker, STREAM, CHANNEL, PERIPHERAL, DIRECTION, BUF>
where
    STREAM: Stream,
    PERIPHERAL: PeriAddress,
{
    pub fn new(
        waker_producer: WakerProducer<'waker>,
        inner: Transfer<STREAM, CHANNEL, PERIPHERAL, DIRECTION, BUF>,
    ) -> Self {
        if inner.double_buf.is_some() {
            panic!("Async SPI doesn't support double buffered operation.");
        }

        Self {
            inner,
            waker_producer,
        }
    }

    pub fn transfer(
        &mut self,
        buffer: BUF,
    ) -> TransferFuture<'_, 'waker, STREAM, CHANNEL, PERIPHERAL, DIRECTION, BUF> {
        TransferFuture {
            inner: self,
            buffer: Some(buffer),
        }
    }
}

pub struct TransferFuture<'transfer, 'waker, STREAM, const CHANNEL: u8, PERIPHERAL, DIRECTION, BUF>
where
    STREAM: Stream,
    PERIPHERAL: PeriAddress,
{
    inner: &'transfer mut AsyncTransfer<'waker, STREAM, CHANNEL, PERIPHERAL, DIRECTION, BUF>,
    buffer: Option<BUF>,
}

impl<'transfer, 'waker, STREAM, const CHANNEL: u8, PERIPHERAL, BUF> core::future::Future
    for TransferFuture<'transfer, 'waker, STREAM, CHANNEL, PERIPHERAL, MemoryToPeripheral, BUF>
where
    STREAM: Stream + Unpin,
    ChannelX<CHANNEL>: Channel,
    PERIPHERAL: PeriAddress + DMASet<STREAM, CHANNEL, MemoryToPeripheral> + Unpin,
    BUF: ReadBuffer<Word = <PERIPHERAL as PeriAddress>::MemSize> + Unpin,
{
    type Output = Result<(), DMAError<BUF>>;

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> Poll<Self::Output> {
        if let Some(buffer) = self.buffer.take() {
            if let Err(e) = self.inner.inner.next_transfer(buffer) {
                return Poll::Ready(Err(e));
            }
        }
        if STREAM::get_fifo_error_flag() {
            self.inner.inner.clear_fifo_error_interrupt();
        }

        if STREAM::get_transfer_error_flag() {
            self.inner.inner.clear_transfer_error_interrupt();
        }

        if STREAM::get_direct_mode_error_flag() {
            self.inner.inner.clear_direct_mode_error_interrupt();
        }

        if STREAM::get_transfer_complete_flag() {
            self.inner.inner.clear_transfer_complete_interrupt();
            return Poll::Ready(Ok(()));
        } else {
            self.inner.waker_producer.enqueue(cx.waker().clone());
            let stream = &mut self.inner.inner.stream;
            cortex_m::interrupt::free(|_| {
                stream.set_interrupts_enable(true, true, true, true);
            });
            Poll::Pending
        }
    }
}
