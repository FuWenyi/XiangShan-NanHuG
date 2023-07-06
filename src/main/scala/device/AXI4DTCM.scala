package device

import chisel3._
import chipsalliance.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy.AddressSet
import utils._

class AXI4DTCM()(implicit p: Parameters) {
  
  val node = AXI4SlaveNode(Seq(AXI4SlavePortParameters(
    Seq(AXI4SlaveParameters(
      address       = List(address),
      regionType    = if (cacheable) RegionType.UNCACHED else RegionType.IDEMPOTENT,
      executable    = executable,
      supportsRead  = TransferSizes(1, beatBytes),
      supportsWrite = TransferSizes(1, beatBytes),
      interleavedId = Some(0))),
    beatBytes  = beatBytes,
    minLatency = 1)))  

  lazy val module = new LazyModuleImp(this) {
    val (in, _) = node.in(0)

    // TODO: DTCM logic
    // to use port(for example): in.ar.bits.addr / in.aw.bits.data / in.b.fire() / in.w.valid / in.r.ready / 
    // AXI bus width: beatBytes
  }
}