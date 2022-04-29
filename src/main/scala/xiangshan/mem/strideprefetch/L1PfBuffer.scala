package xiangshan.mem.strideprefetch
//add by tjz
import chipsalliance.rocketchip.config.{Parameters, Field}
import chisel3._
import chisel3.util._
import utils._
import xiangshan._
import xiangshan.backend.decode.ImmUnion
import xiangshan.cache._
import xiangshan.mem._
//import xiangshan.mem._

class L1pfbPtr(implicit p: Parameters) extends CircularQueuePtr[L1pfbPtr](
    p => p(XSCoreParamsKey).L1dpbSize
){
}

class L1pfbufferReq(implicit p: Parameters) extends XSBundle {
    val vaddr = Vec(L1DPrefetchPipelineWidth, UInt(VAddrBits.W))
    val bufMask = UInt((L1DPrefetchPipelineWidth).W)
}

class L1pfbufferIO(implicit p: Parameters) extends XSBundle {
    val in = Flipped(DecoupledIO(new L1pfbufferReq))
    val out = Vec(L1DPrefetchPipelineWidth, DecoupledIO(new RptResp))
}

class L1PfBuffer(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper {
    val io = IO(new L1pfbufferIO)

    class L1dpBufferEntry(implicit p: Parameters) extends XSBundle {
        val vaddr = UInt(VAddrBits.W)
        //val costTime = UInt(log2Up(L1dpCostTimeMax).W)
    }

    /*val clock_in_buffer = RegInit(0.U(log2Up(L1dpCostTimeMax).W))
    when (clock_in_buffer === L1dpCostTimeMax.U) {
        clock_in_buffer := 0.U
    }.otherwise {
        clock_in_buffer := clock_in_buffer + 1.U
    }*/

    val l1dpbuf = Module(new SyncDataModuleTemplate(new L1dpBufferEntry, L1dpbSize, L1DPrefetchPipelineWidth, L1DPrefetchPipelineWidth))
    val head_vec = RegInit(VecInit((0 until L1DPrefetchPipelineWidth).map(_.U.asTypeOf(new L1pfbPtr))))
    val tail_vec = RegInit(VecInit((0 until L1DPrefetchPipelineWidth).map(_.U.asTypeOf(new L1pfbPtr))))
    val head_ptr = head_vec(0)
    val tail_ptr = tail_vec(0)

    val validEntries = distanceBetween(tail_ptr, head_ptr)
    val full_flush = RegInit(false.B)

    val numberEnq = Mux(io.in.fire, PopCount(io.in.bits.bufMask), 0.U)
    val numberShouldDeq = Mux(validEntries >= L1DPrefetchPipelineWidth.U, L1DPrefetchPipelineWidth.U, validEntries)
    val numberRealDeq = PopCount(io.out.map(_.fire))

    val numberAfterEnq = validEntries +& numberEnq
    val nextValidEntries = Mux(io.out(0).ready, numberAfterEnq - numberShouldDeq, numberAfterEnq)
    // the nextValidEntries are used to control the full signal
    //val nextValidEntries = Mux(io.out(0).ready, numAfterEnq - numberRealDeq, numberAfterEnq)

    //use full signal to control the tail_vec(write ptr),
    full_flush := (L1dpbSize - L1DPrefetchPipelineWidth).U <= nextValidEntries
    //io.in.ready := allowEnq
    io.in.ready := true.B

    // the input stuff
    val offset = Wire(Vec(L1DPrefetchPipelineWidth, UInt(log2Up(L1DPrefetchPipelineWidth).W)))
    for(i <- 0 until L1DPrefetchPipelineWidth) {
        if(i == 0) {
            offset(i) := 0.U
        } else {
            offset(i) := PopCount(io.in.bits.bufMask(i - 1, 0))
        }
    }

    for(i <- 0 until L1DPrefetchPipelineWidth) {
        val inputWire = Wire(new L1dpBufferEntry)
        inputWire.vaddr := io.in.bits.vaddr(i)
        //inputWire.costTime := clock_in_buffer

        l1dpbuf.io.waddr(i) := tail_vec(offset(i)).value
        l1dpbuf.io.wdata(i) := inputWire
        l1dpbuf.io.wen(i) := io.in.bits.bufMask(i) && io.in.fire
    }

    when (io.in.fire && !full_flush) {
        tail_vec := VecInit(tail_vec.map(_ +PopCount(io.in.bits.bufMask)))
        
    }

    // the output stuff
    val validVec = Mux(validEntries >= L1DPrefetchPipelineWidth.U, ((1 << L1DPrefetchPipelineWidth) - 1).U, UIntToMask(validEntries, L1DPrefetchPipelineWidth))

    for (i <- 0 until L1DPrefetchPipelineWidth) {
        io.out(i).valid := validVec(i)
        val outputWire = l1dpbuf.io.rdata(i)

        io.out(i).bits.respVaddr := outputWire.vaddr
        /*when(clock_in_buffer <= outputWire.costTime){
            io.out(i).bits.l1dpBufferCostTime  := L1dpCostTimeMax.U +& clock_in_buffer - outputWire.costTime
        }.otherwise {
            io.out(i).bits.l1dpBufferCostTime  := clock_in_buffer - outputWire.costTime
        }*/
    }

    val next_head_vec = VecInit(head_vec.map(_ + numberRealDeq))
    head_vec := next_head_vec
    l1dpbuf.io.raddr := VecInit(next_head_vec.map(_.value))

    //when the buffer can't enq more data, just flush all
    when (full_flush) {
    val head_vec = VecInit((0 until L1DPrefetchPipelineWidth).map(_.U.asTypeOf(new L1pfbPtr)))
    val tail_vec = VecInit((0 until L1DPrefetchPipelineWidth).map(_.U.asTypeOf(new L1pfbPtr)))
    full_flush := false.B
    }

  when(io.in.fire()) {
    XSDebug(p"prebufMASK=${Binary(io.in.bits.bufMask)} Enque = ${Hexadecimal(numberEnq)}\n")
    for(i <- 0 until L1DPrefetchPipelineWidth){
        XSDebug(io.in.bits.bufMask(i), p"prepaddr=${Hexadecimal(io.in.bits.vaddr(i))}\n")
    }
  }
  

  for (i <- 0 until L1DPrefetchPipelineWidth) {
    XSDebug(io.out(i).fire(), p"deq: reqVaddr=${Hexadecimal(io.out(i).bits.respVaddr)} valid=${io.out(i).valid}\n")
  }

  XSDebug(p"ValidEntries: ${validEntries}\n")
  XSPerfAccumulate("flash_times", full_flush)
}
