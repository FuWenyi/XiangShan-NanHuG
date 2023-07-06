package device

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import chisel3.experimental.ExtModule
import freechips.rocketchip.amba.axi4.{AXI4EdgeParameters, AXI4MasterNode, AXI4SlaveNode}
import freechips.rocketchip.diplomacy.{AddressSet, InModuleBody, LazyModule, LazyModuleImp, RegionType}
import utils.MaskExpand

class AXI4DTCM
(
  address: Seq[AddressSet],
  memByte: Long,
  useBlackBox: Boolean = false,
  executable: Boolean = true,
  beatBytes: Int = 8,
  burstLen: Int = 16,
)(implicit p: Parameters)
  extends AXI4SlaveModule(address, executable, beatBytes, burstLen)
{
  override lazy val module = new AXI4SlaveModuleImp(this){

    val split = beatBytes / 8
    val bankByte = memByte / split
    val offsetBits = log2Up(memByte)

    require(address.length >= 1)
    val baseAddress = address(0).base

    def index(addr: UInt) = ((addr - baseAddress.U)(offsetBits - 1, 0) >> log2Ceil(beatBytes)).asUInt()

    def inRange(addr: UInt) = addr < (baseAddress + memByte).U

    val wIdx = index(waddr) + writeBeatCnt
    val rIdx = index(raddr) + readBeatCnt
    val wen = in.w.fire() && inRange(waddr)
    require(beatBytes >= 8)

    val rdata = if (useBlackBox) {
      val mems = (0 until split).map {_ => Module(new RAMHelper(bankByte))}
      mems.zipWithIndex map { case (mem, i) =>
        mem.clk   := clock
        mem.en    := !reset.asBool() && ((state === s_rdata) || (state === s_wdata))
        mem.rIdx  := (rIdx << log2Up(split)) + i.U
        mem.wIdx  := (wIdx << log2Up(split)) + i.U
        mem.wdata := in.w.bits.data((i + 1) * 64 - 1, i * 64)
        mem.wmask := MaskExpand(in.w.bits.strb((i + 1) * 8 - 1, i * 8))
        mem.wen   := wen
      }
      val rdata = mems.map {mem => mem.rdata}
      Cat(rdata.reverse)
    } else {
      val mem = Mem(memByte / beatBytes, Vec(beatBytes, UInt(8.W)))

      val wdata = VecInit.tabulate(beatBytes) { i => in.w.bits.data(8 * (i + 1) - 1, 8 * i) }
      when(wen) {
        mem.write(wIdx, wdata, in.w.bits.strb.asBools())
      }

      Cat(mem.read(rIdx).reverse)
    }
    in.r.bits.data := rdata
  }
}