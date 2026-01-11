from pynq import Overlay, allocate
import numpy as np

# Load FPGA bitstream (ensure file is flashed or accessible)
ol = Overlay("design.bit")                      # small bitstream path
dma = ol.axi_dma_0                              # DMA AXI IP exposed by overlay

# Prepare input image buffer (uint8 packed) and output buffer
N = 224*224*3                                   # example RGB frame size
in_buf = allocate(shape=(N,), dtype=np.uint8)   # physically contiguous for DMA
out_buf = allocate(shape=(1024,), dtype=np.uint8)

# Fill input buffer (read from camera or sensor DMA into buffer in real systems)
in_buf[:] = np.fromfile("/dev/shm/frame.raw", dtype=np.uint8)[:N]

# Start DMA transfers: send input, then receive output (non-blocking)
dma.sendchannel.transfer(in_buf)
dma.recvchannel.transfer(out_buf)

# Wait for completion and check status
dma.sendchannel.wait()
dma.recvchannel.wait()

# Process results on host CPU or forward to network
result = bytes(out_buf[:128])                   # example: classifier top-k
with open("/tmp/accel_out.bin","wb") as f:
    f.write(result)
# Free buffers (PYNQ handles on exit)