; ModuleID = 'hugr'
source_filename = "hugr"
target datalayout = "e-m:e-p270:32:32-p271:32:32-p272:64:64-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

@res_result.457DE32D.0 = private constant [17 x i8] c"\10USER:BOOL:result"
@"e_No more qu.3B2EEBF0.0" = private constant [47 x i8] c".EXIT:INT:No more qubits available to allocate."

declare i64 @___lazy_measure(i64) local_unnamed_addr

declare void @___qfree(i64) local_unnamed_addr

declare i1 @___read_future_bool(i64) local_unnamed_addr

declare void @___dec_future_refcount(i64) local_unnamed_addr

declare void @print_bool(i8*, i64, i1) local_unnamed_addr

declare i64 @___qalloc() local_unnamed_addr

declare void @___reset(i64) local_unnamed_addr

; Function Attrs: noreturn
declare void @panic(i32, i8*) local_unnamed_addr #0

declare void @___rxy(i64, double, double) local_unnamed_addr

declare void @___rz(i64, double) local_unnamed_addr

declare void @___rzz(i64, i64, double) local_unnamed_addr

define i64 @qmain(i64 %0) local_unnamed_addr {
entry:
  tail call void @setup(i64 %0)
  %qalloc.i.i = tail call i64 @___qalloc()
  %not_max.not.i.i = icmp eq i64 %qalloc.i.i, -1
  br i1 %not_max.not.i.i, label %id_bb.i.i, label %reset_bb.i.i

reset_bb.i.i:                                     ; preds = %entry
  tail call void @___reset(i64 %qalloc.i.i)
  br label %id_bb.i.i

id_bb.i.i:                                        ; preds = %reset_bb.i.i, %entry
  %1 = insertvalue { i1, i64 } { i1 true, i64 poison }, i64 %qalloc.i.i, 1
  %2 = select i1 %not_max.not.i.i, { i1, i64 } { i1 false, i64 poison }, { i1, i64 } %1
  %.fca.0.extract.i.i = extractvalue { i1, i64 } %2, 0
  br i1 %.fca.0.extract.i.i, label %__hugr__.__tk2_qalloc.83.exit.i, label %cond_87_case_0.i.i

cond_87_case_0.i.i:                               ; preds = %id_bb.i.i
  tail call void @panic(i32 1001, i8* getelementptr inbounds ([47 x i8], [47 x i8]* @"e_No more qu.3B2EEBF0.0", i64 0, i64 0))
  unreachable

__hugr__.__tk2_qalloc.83.exit.i:                  ; preds = %id_bb.i.i
  %.fca.1.extract.i.i = extractvalue { i1, i64 } %2, 1
  br label %cond_177_case_1.i.i

cond_177_case_1.i.i:                              ; preds = %cond_177_case_1.i.i.backedge, %__hugr__.__tk2_qalloc.83.exit.i
  %qalloc.i.i.i = tail call i64 @___qalloc()
  %not_max.not.i.i.i = icmp eq i64 %qalloc.i.i.i, -1
  br i1 %not_max.not.i.i.i, label %id_bb.i.i.i, label %reset_bb.i.i.i

reset_bb.i.i.i:                                   ; preds = %cond_177_case_1.i.i
  tail call void @___reset(i64 %qalloc.i.i.i)
  br label %id_bb.i.i.i

id_bb.i.i.i:                                      ; preds = %reset_bb.i.i.i, %cond_177_case_1.i.i
  %3 = insertvalue { i1, i64 } { i1 true, i64 poison }, i64 %qalloc.i.i.i, 1
  %4 = select i1 %not_max.not.i.i.i, { i1, i64 } { i1 false, i64 poison }, { i1, i64 } %3
  %.fca.0.extract.i.i.i = extractvalue { i1, i64 } %4, 0
  br i1 %.fca.0.extract.i.i.i, label %__hugr__.__tk2_qalloc.83.exit.i.i, label %cond_87_case_0.i.i.i

cond_87_case_0.i.i.i:                             ; preds = %id_bb.i.i.i
  tail call void @panic(i32 1001, i8* getelementptr inbounds ([47 x i8], [47 x i8]* @"e_No more qu.3B2EEBF0.0", i64 0, i64 0))
  unreachable

__hugr__.__tk2_qalloc.83.exit.i.i:                ; preds = %id_bb.i.i.i
  %.fca.1.extract.i.i.i = extractvalue { i1, i64 } %4, 1
  %qalloc.i128.i.i = tail call i64 @___qalloc()
  %not_max.not.i129.i.i = icmp eq i64 %qalloc.i128.i.i, -1
  br i1 %not_max.not.i129.i.i, label %id_bb.i132.i.i, label %reset_bb.i130.i.i

reset_bb.i130.i.i:                                ; preds = %__hugr__.__tk2_qalloc.83.exit.i.i
  tail call void @___reset(i64 %qalloc.i128.i.i)
  br label %id_bb.i132.i.i

id_bb.i132.i.i:                                   ; preds = %reset_bb.i130.i.i, %__hugr__.__tk2_qalloc.83.exit.i.i
  %5 = insertvalue { i1, i64 } { i1 true, i64 poison }, i64 %qalloc.i128.i.i, 1
  %6 = select i1 %not_max.not.i129.i.i, { i1, i64 } { i1 false, i64 poison }, { i1, i64 } %5
  %.fca.0.extract.i131.i.i = extractvalue { i1, i64 } %6, 0
  br i1 %.fca.0.extract.i131.i.i, label %__hugr__.__tk2_qalloc.83.exit135.i.i, label %cond_87_case_0.i134.i.i

cond_87_case_0.i134.i.i:                          ; preds = %id_bb.i132.i.i
  tail call void @panic(i32 1001, i8* getelementptr inbounds ([47 x i8], [47 x i8]* @"e_No more qu.3B2EEBF0.0", i64 0, i64 0))
  unreachable

__hugr__.__tk2_qalloc.83.exit135.i.i:             ; preds = %id_bb.i132.i.i
  %.fca.1.extract.i133.i.i = extractvalue { i1, i64 } %6, 1
  tail call void @___rxy(i64 %.fca.1.extract.i133.i.i, double 0x3FF921FB54442D18, double 0xBFF921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i133.i.i, double 0x400921FB54442D18)
  tail call void @___rxy(i64 %.fca.1.extract.i.i.i, double 0x3FF921FB54442D18, double 0xBFF921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i.i.i, double 0x400921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i.i.i, double 0xBFE921FB54442D18)
  tail call void @___rxy(i64 %.fca.1.extract.i.i.i, double 0xBFF921FB54442D18, double 0x3FF921FB54442D18)
  tail call void @___rzz(i64 %.fca.1.extract.i133.i.i, i64 %.fca.1.extract.i.i.i, double 0x3FF921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i133.i.i, double 0xBFF921FB54442D18)
  tail call void @___rxy(i64 %.fca.1.extract.i.i.i, double 0x3FF921FB54442D18, double 0x400921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i.i.i, double 0xBFF921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i.i.i, double 0x3FE921FB54442D18)
  %lazy_measure.i.i = tail call i64 @___lazy_measure(i64 %.fca.1.extract.i.i.i)
  tail call void @___qfree(i64 %.fca.1.extract.i.i.i)
  %read_bool.i.i = tail call i1 @___read_future_bool(i64 %lazy_measure.i.i)
  tail call void @___dec_future_refcount(i64 %lazy_measure.i.i)
  br i1 %read_bool.i.i, label %cond_191_case_1.i.i, label %7

7:                                                ; preds = %__hugr__.__tk2_qalloc.83.exit135.i.i
  tail call void @___qfree(i64 %.fca.1.extract.i133.i.i)
  br label %cond_177_case_1.i.i.backedge

cond_191_case_1.i.i:                              ; preds = %__hugr__.__tk2_qalloc.83.exit135.i.i
  tail call void @___rz(i64 %.fca.1.extract.i.i, double 0x3FE921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i.i, double 0x400921FB54442D18)
  tail call void @___rxy(i64 %.fca.1.extract.i133.i.i, double 0xBFF921FB54442D18, double 0x3FF921FB54442D18)
  tail call void @___rzz(i64 %.fca.1.extract.i.i, i64 %.fca.1.extract.i133.i.i, double 0x3FF921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i.i, double 0xBFF921FB54442D18)
  tail call void @___rxy(i64 %.fca.1.extract.i133.i.i, double 0x3FF921FB54442D18, double 0x400921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i133.i.i, double 0xBFF921FB54442D18)
  tail call void @___rz(i64 %.fca.1.extract.i133.i.i, double 0x3FE921FB54442D18)
  %lazy_measure67.i.i = tail call i64 @___lazy_measure(i64 %.fca.1.extract.i133.i.i)
  tail call void @___qfree(i64 %.fca.1.extract.i133.i.i)
  %read_bool80.i.i = tail call i1 @___read_future_bool(i64 %lazy_measure67.i.i)
  tail call void @___dec_future_refcount(i64 %lazy_measure67.i.i)
  br i1 %read_bool80.i.i, label %__hugr__.main.1.exit, label %8

8:                                                ; preds = %cond_191_case_1.i.i
  tail call void @___rxy(i64 %.fca.1.extract.i.i, double 0x400921FB54442D18, double 0.000000e+00)
  br label %cond_177_case_1.i.i.backedge

cond_177_case_1.i.i.backedge:                     ; preds = %8, %7
  br label %cond_177_case_1.i.i

__hugr__.main.1.exit:                             ; preds = %cond_191_case_1.i.i
  %lazy_measure.i = tail call i64 @___lazy_measure(i64 %.fca.1.extract.i.i)
  tail call void @___qfree(i64 %.fca.1.extract.i.i)
  %read_bool.i = tail call i1 @___read_future_bool(i64 %lazy_measure.i)
  tail call void @___dec_future_refcount(i64 %lazy_measure.i)
  tail call void @print_bool(i8* getelementptr inbounds ([17 x i8], [17 x i8]* @res_result.457DE32D.0, i64 0, i64 0), i64 16, i1 %read_bool.i)
  %9 = tail call i64 @teardown()
  ret i64 %9
}

declare void @setup(i64) local_unnamed_addr

declare i64 @teardown() local_unnamed_addr

attributes #0 = { noreturn }

!name = !{!0}

!0 = !{!"mainlib"}
