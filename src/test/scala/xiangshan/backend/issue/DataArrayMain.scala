package xiangshan.backend.issue

import chisel3._
import freechips.rocketchip.diplomacy.DisableMonitors
import top.{ArgParser, BaseConfig, Generator}
import xiangshan.{XSCoreParameters, XSCoreParamsKey}

object DataArrayMain extends App {
  override def main(args: Array[String]): Unit = {
    val (config, firrtlOpts, firrtlComplier) = ArgParser.parse(args)
    val backendParams = config(XSCoreParamsKey).backendParams

    val iqParams: IssueBlockParams = backendParams.intSchdParams.get.issueBlockParams.head

    Generator.execute(
      firrtlOpts,
      // DataArray
      DisableMonitors(p =>
        new DataArray(Vec(iqParams.dataBitsMax, Bool()), iqParams.numDeq, iqParams.numEnq, iqParams.numEntries)(
          p.alterPartial({
            case XSCoreParamsKey => XSCoreParameters()
          })))(config),
      firrtlComplier
    )
  }
}