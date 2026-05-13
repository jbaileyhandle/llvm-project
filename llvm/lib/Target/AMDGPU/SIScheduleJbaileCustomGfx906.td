//===-- SIScheduleJbaileCustomGfx906.td - jbaile custom gfx906 model ----===//
// Phase 1: verbatim copy of SIQuarterSpeedModel structure, renamed.

def SIGfx906JbaileCustomModel : SISchedMachineModel;

let SchedModel = SIGfx906JbaileCustomModel in {

defm : SICommonWriteRes;

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
