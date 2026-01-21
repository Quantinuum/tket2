; ModuleID = 'hugr'
source_filename = "hugr"
target datalayout = "e-m:e-p270:32:32-p271:32:32-p272:64:64-i64:64-f80:128-n8:16:32:64-S128"
target triple = "x86_64-unknown-linux-gnu"

@res_rint.B928E41E.0 = private constant [14 x i8] c"\0DUSER:INT:rint"
@res_rint1.0884EC03.0 = private constant [15 x i8] c"\0EUSER:INT:rint1"
@res_rfloat.F0E4DD2C.0 = private constant [18 x i8] c"\11USER:FLOAT:rfloat"
@res_rint_bnd.CB1E6B0D.0 = private constant [18 x i8] c"\11USER:INT:rint_bnd"
@res_rint2.F0335598.0 = private constant [15 x i8] c"\0EUSER:INT:rint2"
@res_rfloat2.4DAB941F.0 = private constant [19 x i8] c"\12USER:FLOAT:rfloat2"
@res_rint_bnd2.169DE399.0 = private constant [19 x i8] c"\12USER:INT:rint_bnd2"

declare i32 @random_int() local_unnamed_addr

declare double @random_float() local_unnamed_addr

declare i32 @random_rng(i32) local_unnamed_addr

declare void @print_int(i8*, i64, i64) local_unnamed_addr

declare void @print_float(i8*, i64, double) local_unnamed_addr

declare void @random_seed(i64) local_unnamed_addr

define i64 @qmain(i64 %0) local_unnamed_addr {
entry:
  tail call void @setup(i64 %0)
  tail call void @random_seed(i64 42)
  %rint.i = tail call i32 @random_int()
  %rint20.i = tail call i32 @random_int()
  %rfloat.i = tail call double @random_float()
  %rintb.i = tail call i32 @random_rng(i32 100)
  %1 = sext i32 %rintb.i to i64
  %2 = sext i32 %rint20.i to i64
  %3 = sext i32 %rint.i to i64
  tail call void @print_int(i8* getelementptr inbounds ([14 x i8], [14 x i8]* @res_rint.B928E41E.0, i64 0, i64 0), i64 13, i64 %3)
  tail call void @print_int(i8* getelementptr inbounds ([15 x i8], [15 x i8]* @res_rint1.0884EC03.0, i64 0, i64 0), i64 14, i64 %2)
  tail call void @print_float(i8* getelementptr inbounds ([18 x i8], [18 x i8]* @res_rfloat.F0E4DD2C.0, i64 0, i64 0), i64 17, double %rfloat.i)
  tail call void @print_int(i8* getelementptr inbounds ([18 x i8], [18 x i8]* @res_rint_bnd.CB1E6B0D.0, i64 0, i64 0), i64 17, i64 %1)
  tail call void @random_seed(i64 84)
  %rint53.i = tail call i32 @random_int()
  %rfloat55.i = tail call double @random_float()
  %rintb58.i = tail call i32 @random_rng(i32 200)
  %4 = sext i32 %rintb58.i to i64
  %5 = sext i32 %rint53.i to i64
  tail call void @print_int(i8* getelementptr inbounds ([15 x i8], [15 x i8]* @res_rint2.F0335598.0, i64 0, i64 0), i64 14, i64 %5)
  tail call void @print_float(i8* getelementptr inbounds ([19 x i8], [19 x i8]* @res_rfloat2.4DAB941F.0, i64 0, i64 0), i64 18, double %rfloat55.i)
  tail call void @print_int(i8* getelementptr inbounds ([19 x i8], [19 x i8]* @res_rint_bnd2.169DE399.0, i64 0, i64 0), i64 18, i64 %4)
  %6 = tail call i64 @teardown()
  ret i64 %6
}

declare void @setup(i64) local_unnamed_addr

declare i64 @teardown() local_unnamed_addr

!name = !{!0}

!0 = !{!"mainlib"}
