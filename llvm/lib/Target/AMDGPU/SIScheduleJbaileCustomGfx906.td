//===-- SIScheduleJbaileCustomGfx906.td - jbaile custom gfx906 models ---===//
// Runtime-swappable alternate gfx906 sched models, selected via misched.txt's
// `timing_model=<name>` option (see AMDGPUSubtarget). Each model is bound to a
// synthetic processor in GCNProcessors.td so TableGen emits it; end users still
// compile with -mcpu=gfx906.
//
// Both models share one recipe (JbaileGfx906ModelBody): the gfx906 baseline
// (SIQuarterSpeedModel) with the global-memory latency left as a blank to fill
// in. Each model just fills that blank -- so every model is an exact copy of the
// baseline except the one latency it overrides, with no duplicated tables.

// The gfx906 baseline recipe, with global-memory (VMEM) latency as the blank.
// SICommonWriteRes<VMEMLatency> supplies the common writes (VMEM = VMEMLatency);
// the rest is SIQuarterSpeedModel's VALU/InstRW specifics verbatim.
multiclass JbaileGfx906ModelBody<int VMEMLatency> {
  defm : SICommonWriteRes<VMEMLatency>;

  let RetireOOO = 1 in {
  def : HWVALUWriteRes<Write64Bit,       2>;
  def : HWVALUWriteRes<WriteIntMul,      4>;
  def : HWVALUWriteRes<WriteFloatFMA,    16>;
  def : HWVALUWriteRes<WriteDouble,      16>;
  def : HWVALUWriteRes<WriteDoubleAdd,    8>;
  def : HWVALUWriteRes<WriteDoubleCvt,    4>;
  def : HWVALUWriteRes<WriteTrans64,     16>;
  }

  def : InstRW<[WriteCopy], (instrs COPY)>;
  def : InstRW<[WriteCopy], (instrs PRED_COPY)>;
  def : InstRW<[Write64Bit, MIReadVGPR], (instregex "^V_ACCVGPR_WRITE_B32_e64$")>;
  def : InstRW<[Write2PassMAI,  MIMFMARead], (instregex "^V_MFMA_..._4X4X")>;
  def : InstRW<[Write8PassMAI,  MIMFMARead], (instregex "^V_MFMA_..._16X16X")>;
  def : InstRW<[Write16PassMAI, MIMFMARead], (instregex "^V_MFMA_..._32X32X")>;
}

// `timing_model=custom`: passthrough -- byte-identical to the in-tree gfx906
// model (VMEM = 80). Kept as an A/B control for the swap mechanism.
def SIGfx906JbaileCustomModel : SISchedMachineModel;
let SchedModel = SIGfx906JbaileCustomModel in {
  defm : JbaileGfx906ModelBody<80>;
}

// `timing_model=fast_memory`: baseline with global-memory latency lowered to LDS
// access latency (5). Everything else identical to the baseline.
def SIGfx906JbaileFastMemModel : SISchedMachineModel;
let SchedModel = SIGfx906JbaileFastMemModel in {
  defm : JbaileGfx906ModelBody<5>;
}
