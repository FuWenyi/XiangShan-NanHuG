package device

import chisel3._
import chipsalliance.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy.AddressSet
import utils._
import freechips.rocketchip.amba.axi4.AXI4SlaveNode
import freechips.rocketchip.amba.axi4.AXI4SlavePortParameters
import freechips.rocketchip.amba.axi4.AXI4SlaveParameters
import freechips.rocketchip.diplomacy.RegionType
import freechips.rocketchip.diplomacy.TransferSizes
import freechips.rocketchip.diplomacy.LazyModuleImp
import freechips.rocketchip.diplomacy.LazyModule

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

  private val sram = LazyModule(new AXI4RAM(
    address=Seq(AddressSet(0x8,   ~0x808)),//Seq[0x20000, 0x3ff],
    memByte= 4096,
    executable = false
  ))  
  sram.node := node
}