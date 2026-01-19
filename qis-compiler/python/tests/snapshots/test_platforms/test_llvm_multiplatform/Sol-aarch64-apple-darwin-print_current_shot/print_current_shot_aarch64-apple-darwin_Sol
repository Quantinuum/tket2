; ModuleID = 'hugr'
source_filename = "hugr"
target datalayout = "e-m:o-i64:64-i128:128-n32:64-S128"
target triple = "aarch64-apple-darwin"

@res_shot.6D86EAF7.0 = private constant [14 x i8] c"\0DUSER:INT:shot"

declare i64 @get_current_shot() local_unnamed_addr

declare void @print_int(i8*, i64, i64) local_unnamed_addr

define i64 @qmain(i64 %0) local_unnamed_addr {
entry:
  tail call void @setup(i64 %0)
  %shot.i = tail call i64 @get_current_shot()
  tail call void @print_int(i8* getelementptr inbounds ([14 x i8], [14 x i8]* @res_shot.6D86EAF7.0, i64 0, i64 0), i64 13, i64 %shot.i)
  %1 = tail call i64 @teardown()
  ret i64 %1
}

declare void @setup(i64) local_unnamed_addr

declare i64 @teardown() local_unnamed_addr

!name = !{!0}

!0 = !{!"mainlib"}
