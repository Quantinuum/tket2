; ModuleID = 'hugr'
source_filename = "hugr"
target datalayout = "e-m:o-i64:64-i128:128-n32:64-S128"
target triple = "aarch64-apple-darwin"

@"e_Array alre.5A300C2A.0" = private constant [57 x i8] c"8EXIT:INT:Array already contains an element at this index"
@"e_Array elem.E746B1A3.0" = private constant [43 x i8] c"*EXIT:INT:Array element is already borrowed"
@"e_Some array.A77EF32E.0" = private constant [48 x i8] c"/EXIT:INT:Some array elements have been borrowed"
@"e_Array cont.EFA5AC45.0" = private constant [70 x i8] c"EEXIT:INT:Array contains non-borrowed elements and cannot be discarded"
@res_cs.46C3C4B5.0 = private constant [16 x i8] c"\0FUSER:BOOLARR:cs"
@res_is.F21393DB.0 = private constant [15 x i8] c"\0EUSER:INTARR:is"
@res_fs.CBD4AF54.0 = private constant [17 x i8] c"\10USER:FLOATARR:fs"
@"e_No more qu.3B2EEBF0.0" = private constant [47 x i8] c".EXIT:INT:No more qubits available to allocate."
@"e_Expected v.E6312129.0" = private constant [46 x i8] c"-EXIT:INT:Expected variant 1 but got variant 0"
@"e_Expected v.2F17E0A9.0" = private constant [46 x i8] c"-EXIT:INT:Expected variant 0 but got variant 1"

define private fastcc void @__hugr__.main.1() unnamed_addr {
alloca_block:
  %0 = tail call i8* @heap_alloc(i64 800)
  %1 = bitcast i8* %0 to double*
  %2 = tail call i8* @heap_alloc(i64 16)
  %3 = bitcast i8* %2 to i64*
  tail call void @llvm.memset.p0i64.i64(i64* noundef nonnull align 1 dereferenceable(16) %3, i8 -1, i64 16, i1 false)
  %4 = tail call i8* @heap_alloc(i64 800)
  %5 = bitcast i8* %4 to i64*
  %6 = tail call i8* @heap_alloc(i64 16)
  %7 = bitcast i8* %6 to i64*
  tail call void @llvm.memset.p0i64.i64(i64* noundef nonnull align 1 dereferenceable(16) %7, i8 -1, i64 16, i1 false)
  %8 = tail call i8* @heap_alloc(i64 80)
  %9 = bitcast i8* %8 to i64*
  %10 = tail call i8* @heap_alloc(i64 8)
  %11 = bitcast i8* %10 to i64*
  store i64 -1, i64* %11, align 1
  br label %cond_20_case_1

cond_20_case_1:                                   ; preds = %alloca_block, %cond_exit_20
  %"15_0.sroa.0.0961" = phi i64 [ 0, %alloca_block ], [ %12, %cond_exit_20 ]
  %12 = add nuw nsw i64 %"15_0.sroa.0.0961", 1
  %qalloc.i = tail call i64 @___qalloc()
  %not_max.not.i = icmp eq i64 %qalloc.i, -1
  br i1 %not_max.not.i, label %id_bb.i, label %reset_bb.i

reset_bb.i:                                       ; preds = %cond_20_case_1
  tail call void @___reset(i64 %qalloc.i)
  br label %id_bb.i

id_bb.i:                                          ; preds = %reset_bb.i, %cond_20_case_1
  %13 = insertvalue { i1, i64 } { i1 true, i64 poison }, i64 %qalloc.i, 1
  %14 = select i1 %not_max.not.i, { i1, i64 } { i1 false, i64 poison }, { i1, i64 } %13
  %.fca.0.extract.i = extractvalue { i1, i64 } %14, 0
  br i1 %.fca.0.extract.i, label %__barray_check_bounds.exit, label %cond_303_case_0.i

cond_303_case_0.i:                                ; preds = %id_bb.i
  tail call void @panic(i32 1001, i8* getelementptr inbounds ([47 x i8], [47 x i8]* @"e_No more qu.3B2EEBF0.0", i64 0, i64 0))
  unreachable

__barray_check_bounds.exit:                       ; preds = %id_bb.i
  %15 = lshr i64 %"15_0.sroa.0.0961", 6
  %16 = getelementptr inbounds i64, i64* %11, i64 %15
  %17 = load i64, i64* %16, align 4
  %18 = shl nuw nsw i64 1, %"15_0.sroa.0.0961"
  %19 = and i64 %17, %18
  %.not.i = icmp eq i64 %19, 0
  br i1 %.not.i, label %panic.i, label %cond_exit_20

panic.i:                                          ; preds = %__barray_check_bounds.exit
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([57 x i8], [57 x i8]* @"e_Array alre.5A300C2A.0", i64 0, i64 0))
  unreachable

cond_exit_20:                                     ; preds = %__barray_check_bounds.exit
  %.fca.1.extract.i = extractvalue { i1, i64 } %14, 1
  %20 = xor i64 %17, %18
  store i64 %20, i64* %16, align 4
  %21 = getelementptr inbounds i64, i64* %9, i64 %"15_0.sroa.0.0961"
  store i64 %.fca.1.extract.i, i64* %21, align 4
  %exitcond.not = icmp eq i64 %12, 10
  br i1 %exitcond.not, label %loop_out, label %cond_20_case_1

loop_out:                                         ; preds = %cond_exit_20
  %22 = load i64, i64* %11, align 4
  %23 = and i64 %22, 1
  %.not.i852 = icmp eq i64 %23, 0
  br i1 %.not.i852, label %__barray_mask_borrow.exit, label %panic.i853

panic.i853:                                       ; preds = %loop_out
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([43 x i8], [43 x i8]* @"e_Array elem.E746B1A3.0", i64 0, i64 0))
  unreachable

__barray_mask_borrow.exit:                        ; preds = %loop_out
  %24 = xor i64 %22, 1
  store i64 %24, i64* %11, align 4
  %25 = load i64, i64* %9, align 4
  tail call void @___rxy(i64 %25, double 0x400921FB54442D18, double 0.000000e+00)
  %26 = load i64, i64* %11, align 4
  %27 = and i64 %26, 1
  %.not.i854 = icmp eq i64 %27, 0
  br i1 %.not.i854, label %panic.i855, label %__barray_mask_return.exit856

panic.i855:                                       ; preds = %__barray_mask_borrow.exit
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([57 x i8], [57 x i8]* @"e_Array alre.5A300C2A.0", i64 0, i64 0))
  unreachable

__barray_mask_return.exit856:                     ; preds = %__barray_mask_borrow.exit
  %28 = xor i64 %26, 1
  store i64 %28, i64* %11, align 4
  store i64 %25, i64* %9, align 4
  %29 = load i64, i64* %11, align 4
  %30 = and i64 %29, 4
  %.not.i857 = icmp eq i64 %30, 0
  br i1 %.not.i857, label %__barray_mask_borrow.exit859, label %panic.i858

panic.i858:                                       ; preds = %__barray_mask_return.exit856
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([43 x i8], [43 x i8]* @"e_Array elem.E746B1A3.0", i64 0, i64 0))
  unreachable

__barray_mask_borrow.exit859:                     ; preds = %__barray_mask_return.exit856
  %31 = xor i64 %29, 4
  store i64 %31, i64* %11, align 4
  %32 = getelementptr inbounds i8, i8* %8, i64 16
  %33 = bitcast i8* %32 to i64*
  %34 = load i64, i64* %33, align 4
  tail call void @___rxy(i64 %34, double 0x400921FB54442D18, double 0.000000e+00)
  %35 = load i64, i64* %11, align 4
  %36 = and i64 %35, 4
  %.not.i860 = icmp eq i64 %36, 0
  br i1 %.not.i860, label %panic.i861, label %__barray_mask_return.exit862

panic.i861:                                       ; preds = %__barray_mask_borrow.exit859
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([57 x i8], [57 x i8]* @"e_Array alre.5A300C2A.0", i64 0, i64 0))
  unreachable

__barray_mask_return.exit862:                     ; preds = %__barray_mask_borrow.exit859
  %37 = xor i64 %35, 4
  store i64 %37, i64* %11, align 4
  store i64 %34, i64* %33, align 4
  %38 = load i64, i64* %11, align 4
  %39 = and i64 %38, 8
  %.not.i863 = icmp eq i64 %39, 0
  br i1 %.not.i863, label %__barray_mask_borrow.exit865, label %panic.i864

panic.i864:                                       ; preds = %__barray_mask_return.exit862
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([43 x i8], [43 x i8]* @"e_Array elem.E746B1A3.0", i64 0, i64 0))
  unreachable

__barray_mask_borrow.exit865:                     ; preds = %__barray_mask_return.exit862
  %40 = xor i64 %38, 8
  store i64 %40, i64* %11, align 4
  %41 = getelementptr inbounds i8, i8* %8, i64 24
  %42 = bitcast i8* %41 to i64*
  %43 = load i64, i64* %42, align 4
  tail call void @___rxy(i64 %43, double 0x400921FB54442D18, double 0.000000e+00)
  %44 = load i64, i64* %11, align 4
  %45 = and i64 %44, 8
  %.not.i866 = icmp eq i64 %45, 0
  br i1 %.not.i866, label %panic.i867, label %__barray_mask_return.exit868

panic.i867:                                       ; preds = %__barray_mask_borrow.exit865
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([57 x i8], [57 x i8]* @"e_Array alre.5A300C2A.0", i64 0, i64 0))
  unreachable

__barray_mask_return.exit868:                     ; preds = %__barray_mask_borrow.exit865
  %46 = xor i64 %44, 8
  store i64 %46, i64* %11, align 4
  store i64 %43, i64* %42, align 4
  %47 = load i64, i64* %11, align 4
  %48 = and i64 %47, 512
  %.not.i869 = icmp eq i64 %48, 0
  br i1 %.not.i869, label %__barray_mask_borrow.exit871, label %panic.i870

panic.i870:                                       ; preds = %__barray_mask_return.exit868
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([43 x i8], [43 x i8]* @"e_Array elem.E746B1A3.0", i64 0, i64 0))
  unreachable

__barray_mask_borrow.exit871:                     ; preds = %__barray_mask_return.exit868
  %49 = xor i64 %47, 512
  store i64 %49, i64* %11, align 4
  %50 = getelementptr inbounds i8, i8* %8, i64 72
  %51 = bitcast i8* %50 to i64*
  %52 = load i64, i64* %51, align 4
  tail call void @___rxy(i64 %52, double 0x400921FB54442D18, double 0.000000e+00)
  %53 = load i64, i64* %11, align 4
  %54 = and i64 %53, 512
  %.not.i872 = icmp eq i64 %54, 0
  br i1 %.not.i872, label %panic.i873, label %__barray_mask_return.exit874

panic.i873:                                       ; preds = %__barray_mask_borrow.exit871
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([57 x i8], [57 x i8]* @"e_Array alre.5A300C2A.0", i64 0, i64 0))
  unreachable

__barray_mask_return.exit874:                     ; preds = %__barray_mask_borrow.exit871
  %55 = xor i64 %53, 512
  store i64 %55, i64* %11, align 4
  store i64 %52, i64* %51, align 4
  %56 = tail call i8* @heap_alloc(i64 240)
  %57 = bitcast i8* %56 to { i1, i64, i1 }*
  %58 = tail call i8* @heap_alloc(i64 8)
  %59 = bitcast i8* %58 to i64*
  store i64 -1, i64* %59, align 1
  br label %69

mask_block_ok.i.i.i:                              ; preds = %cond_exit_443.i
  %60 = load i64, i64* %11, align 4
  %61 = or i64 %60, -1024
  store i64 %61, i64* %11, align 4
  %62 = icmp eq i64 %61, -1
  br i1 %62, label %"__hugr__.$measure_array$$n(10).367.exit", label %mask_block_err.i.i.i

"__hugr__.$measure_array$$n(10).367.exit":        ; preds = %mask_block_ok.i.i.i
  tail call void @heap_free(i8* nonnull %8)
  tail call void @heap_free(i8* nonnull %10)
  %63 = tail call i8* @heap_alloc(i64 320)
  %64 = tail call i8* @heap_alloc(i64 8)
  %65 = bitcast i8* %64 to i64*
  store i64 0, i64* %65, align 1
  call void @llvm.memset.p0i8.i64(i8* noundef nonnull align 4 dereferenceable(320) %63, i8 0, i64 320, i1 false)
  %66 = load i64, i64* %59, align 4
  %67 = and i64 %66, 1023
  store i64 %67, i64* %59, align 4
  %68 = icmp eq i64 %67, 0
  br i1 %68, label %__barray_check_none_borrowed.exit, label %mask_block_err.i

mask_block_err.i.i.i:                             ; preds = %mask_block_ok.i.i.i
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([70 x i8], [70 x i8]* @"e_Array cont.EFA5AC45.0", i64 0, i64 0))
  unreachable

69:                                               ; preds = %__barray_mask_return.exit874, %cond_exit_443.i
  %"393_0.sroa.15.0.i963" = phi i64 [ 0, %__barray_mask_return.exit874 ], [ %70, %cond_exit_443.i ]
  %70 = add nuw nsw i64 %"393_0.sroa.15.0.i963", 1
  %71 = lshr i64 %"393_0.sroa.15.0.i963", 6
  %72 = getelementptr inbounds i64, i64* %11, i64 %71
  %73 = load i64, i64* %72, align 4
  %74 = shl nuw nsw i64 1, %"393_0.sroa.15.0.i963"
  %75 = and i64 %73, %74
  %.not.i99.i.i = icmp eq i64 %75, 0
  br i1 %.not.i99.i.i, label %__barray_check_bounds.exit.i, label %panic.i.i.i

panic.i.i.i:                                      ; preds = %69
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([43 x i8], [43 x i8]* @"e_Array elem.E746B1A3.0", i64 0, i64 0))
  unreachable

__barray_check_bounds.exit.i:                     ; preds = %69
  %76 = xor i64 %73, %74
  store i64 %76, i64* %72, align 4
  %77 = getelementptr inbounds i64, i64* %9, i64 %"393_0.sroa.15.0.i963"
  %78 = load i64, i64* %77, align 4
  %lazy_measure.i = tail call i64 @___lazy_measure(i64 %78)
  tail call void @___qfree(i64 %78)
  %79 = getelementptr inbounds i64, i64* %59, i64 %71
  %80 = load i64, i64* %79, align 4
  %81 = and i64 %80, %74
  %.not.i.i = icmp eq i64 %81, 0
  br i1 %.not.i.i, label %panic.i.i, label %cond_exit_443.i

panic.i.i:                                        ; preds = %__barray_check_bounds.exit.i
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([57 x i8], [57 x i8]* @"e_Array alre.5A300C2A.0", i64 0, i64 0))
  unreachable

cond_exit_443.i:                                  ; preds = %__barray_check_bounds.exit.i
  %"457_054.fca.1.insert.i" = insertvalue { i1, i64, i1 } { i1 true, i64 poison, i1 poison }, i64 %lazy_measure.i, 1
  %82 = xor i64 %80, %74
  store i64 %82, i64* %79, align 4
  %83 = getelementptr inbounds { i1, i64, i1 }, { i1, i64, i1 }* %57, i64 %"393_0.sroa.15.0.i963"
  store { i1, i64, i1 } %"457_054.fca.1.insert.i", { i1, i64, i1 }* %83, align 4
  %exitcond979.not = icmp eq i64 %70, 10
  br i1 %exitcond979.not, label %mask_block_ok.i.i.i, label %69

__barray_check_none_borrowed.exit:                ; preds = %"__hugr__.$measure_array$$n(10).367.exit"
  %84 = tail call i8* @heap_alloc(i64 240)
  %85 = bitcast i8* %84 to { i1, i64, i1 }*
  %86 = tail call i8* @heap_alloc(i64 8)
  %87 = bitcast i8* %86 to i64*
  store i64 0, i64* %87, align 1
  %88 = bitcast i8* %63 to { i1, { i1, i64, i1 } }*
  br label %89

mask_block_err.i:                                 ; preds = %"__hugr__.$measure_array$$n(10).367.exit"
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([48 x i8], [48 x i8]* @"e_Some array.A77EF32E.0", i64 0, i64 0))
  unreachable

89:                                               ; preds = %__barray_check_none_borrowed.exit, %__hugr__.const_fun_290.309.exit
  %storemerge850968 = phi i64 [ 0, %__barray_check_none_borrowed.exit ], [ %107, %__hugr__.const_fun_290.309.exit ]
  %90 = phi i64 [ 0, %__barray_check_none_borrowed.exit ], [ %105, %__hugr__.const_fun_290.309.exit ]
  %91 = getelementptr inbounds { i1, i64, i1 }, { i1, i64, i1 }* %57, i64 %storemerge850968
  %92 = load { i1, i64, i1 }, { i1, i64, i1 }* %91, align 4
  %.fca.0.extract118.i = extractvalue { i1, i64, i1 } %92, 0
  %.fca.1.extract119.i = extractvalue { i1, i64, i1 } %92, 1
  br i1 %.fca.0.extract118.i, label %cond_525_case_1.i, label %cond_exit_525.i

cond_525_case_1.i:                                ; preds = %89
  tail call void @___inc_future_refcount(i64 %.fca.1.extract119.i)
  %93 = insertvalue { i1, i64, i1 } { i1 true, i64 poison, i1 poison }, i64 %.fca.1.extract119.i, 1
  br label %cond_exit_525.i

cond_exit_525.i:                                  ; preds = %cond_525_case_1.i, %89
  %.pn.i = phi { i1, i64, i1 } [ %93, %cond_525_case_1.i ], [ %92, %89 ]
  %"04.sroa.6.0.i" = extractvalue { i1, i64, i1 } %.pn.i, 2
  %exitcond980.not = icmp eq i64 %storemerge850968, 10
  br i1 %exitcond980.not, label %cond_528_case_0.i, label %94

94:                                               ; preds = %cond_exit_525.i
  %95 = lshr i64 %90, 6
  %96 = getelementptr inbounds i64, i64* %65, i64 %95
  %97 = load i64, i64* %96, align 4
  %98 = and i64 %90, 63
  %99 = shl nuw i64 1, %98
  %100 = and i64 %97, %99
  %.not.i.i876 = icmp eq i64 %100, 0
  br i1 %.not.i.i876, label %cond_528_case_1.i, label %panic.i.i877

panic.i.i877:                                     ; preds = %94
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([43 x i8], [43 x i8]* @"e_Array elem.E746B1A3.0", i64 0, i64 0))
  unreachable

cond_528_case_0.i:                                ; preds = %cond_exit_525.i
  tail call void @panic(i32 1001, i8* getelementptr inbounds ([46 x i8], [46 x i8]* @"e_Expected v.E6312129.0", i64 0, i64 0))
  unreachable

cond_528_case_1.i:                                ; preds = %94
  %"17.fca.2.insert.i" = insertvalue { i1, i64, i1 } %92, i1 %"04.sroa.6.0.i", 2
  %101 = insertvalue { i1, { i1, i64, i1 } } { i1 true, { i1, i64, i1 } poison }, { i1, i64, i1 } %"17.fca.2.insert.i", 1
  %102 = getelementptr inbounds { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %88, i64 %90
  %103 = getelementptr inbounds { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %102, i64 0, i32 0
  %104 = load i1, i1* %103, align 1
  store { i1, { i1, i64, i1 } } %101, { i1, { i1, i64, i1 } }* %102, align 4
  br i1 %104, label %cond_529_case_1.i, label %__hugr__.const_fun_290.309.exit

cond_529_case_1.i:                                ; preds = %cond_528_case_1.i
  tail call void @panic(i32 1001, i8* getelementptr inbounds ([46 x i8], [46 x i8]* @"e_Expected v.2F17E0A9.0", i64 0, i64 0))
  unreachable

__hugr__.const_fun_290.309.exit:                  ; preds = %cond_528_case_1.i
  %105 = add nuw nsw i64 %90, 1
  %106 = getelementptr inbounds { i1, i64, i1 }, { i1, i64, i1 }* %85, i64 %storemerge850968
  store { i1, i64, i1 } %"17.fca.2.insert.i", { i1, i64, i1 }* %106, align 4
  %107 = add nuw nsw i64 %storemerge850968, 1
  %exitcond981.not = icmp eq i64 %107, 10
  br i1 %exitcond981.not, label %mask_block_ok.i881, label %89

mask_block_ok.i881:                               ; preds = %__hugr__.const_fun_290.309.exit
  tail call void @heap_free(i8* nonnull %56)
  tail call void @heap_free(i8* %58)
  %108 = load i64, i64* %65, align 4
  %109 = and i64 %108, 1023
  store i64 %109, i64* %65, align 4
  %110 = icmp eq i64 %109, 0
  br i1 %110, label %__barray_check_none_borrowed.exit883, label %mask_block_err.i882

mask_block_err.i882:                              ; preds = %mask_block_ok.i881
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([48 x i8], [48 x i8]* @"e_Some array.A77EF32E.0", i64 0, i64 0))
  unreachable

__barray_check_none_borrowed.exit883:             ; preds = %mask_block_ok.i881
  %111 = tail call i8* @heap_alloc(i64 240)
  %112 = bitcast i8* %111 to { i1, i64, i1 }*
  %113 = tail call i8* @heap_alloc(i64 8)
  %114 = bitcast i8* %113 to i64*
  store i64 0, i64* %114, align 1
  %115 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %88, align 4
  %.fca.0.extract11.i = extractvalue { i1, { i1, i64, i1 } } %115, 0
  br i1 %.fca.0.extract11.i, label %__hugr__.const_fun_284.290.exit, label %cond_570_case_0.i

cond_570_case_0.i:                                ; preds = %__hugr__.const_fun_284.290.exit.8, %__hugr__.const_fun_284.290.exit.7, %__hugr__.const_fun_284.290.exit.6, %__hugr__.const_fun_284.290.exit.5, %__hugr__.const_fun_284.290.exit.4, %__hugr__.const_fun_284.290.exit.3, %__hugr__.const_fun_284.290.exit.2, %__hugr__.const_fun_284.290.exit.1, %__hugr__.const_fun_284.290.exit, %__barray_check_none_borrowed.exit883
  tail call void @panic(i32 1001, i8* getelementptr inbounds ([46 x i8], [46 x i8]* @"e_Expected v.E6312129.0", i64 0, i64 0))
  unreachable

__hugr__.const_fun_284.290.exit:                  ; preds = %__barray_check_none_borrowed.exit883
  %116 = extractvalue { i1, { i1, i64, i1 } } %115, 1
  store { i1, i64, i1 } %116, { i1, i64, i1 }* %112, align 4
  %117 = getelementptr inbounds i8, i8* %63, i64 32
  %118 = bitcast i8* %117 to { i1, { i1, i64, i1 } }*
  %119 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %118, align 4
  %.fca.0.extract11.i.1 = extractvalue { i1, { i1, i64, i1 } } %119, 0
  br i1 %.fca.0.extract11.i.1, label %__hugr__.const_fun_284.290.exit.1, label %cond_570_case_0.i

__hugr__.const_fun_284.290.exit.1:                ; preds = %__hugr__.const_fun_284.290.exit
  %120 = extractvalue { i1, { i1, i64, i1 } } %119, 1
  %121 = getelementptr inbounds i8, i8* %111, i64 24
  %122 = bitcast i8* %121 to { i1, i64, i1 }*
  store { i1, i64, i1 } %120, { i1, i64, i1 }* %122, align 4
  %123 = getelementptr inbounds i8, i8* %63, i64 64
  %124 = bitcast i8* %123 to { i1, { i1, i64, i1 } }*
  %125 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %124, align 4
  %.fca.0.extract11.i.2 = extractvalue { i1, { i1, i64, i1 } } %125, 0
  br i1 %.fca.0.extract11.i.2, label %__hugr__.const_fun_284.290.exit.2, label %cond_570_case_0.i

__hugr__.const_fun_284.290.exit.2:                ; preds = %__hugr__.const_fun_284.290.exit.1
  %126 = extractvalue { i1, { i1, i64, i1 } } %125, 1
  %127 = getelementptr inbounds i8, i8* %111, i64 48
  %128 = bitcast i8* %127 to { i1, i64, i1 }*
  store { i1, i64, i1 } %126, { i1, i64, i1 }* %128, align 4
  %129 = getelementptr inbounds i8, i8* %63, i64 96
  %130 = bitcast i8* %129 to { i1, { i1, i64, i1 } }*
  %131 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %130, align 4
  %.fca.0.extract11.i.3 = extractvalue { i1, { i1, i64, i1 } } %131, 0
  br i1 %.fca.0.extract11.i.3, label %__hugr__.const_fun_284.290.exit.3, label %cond_570_case_0.i

__hugr__.const_fun_284.290.exit.3:                ; preds = %__hugr__.const_fun_284.290.exit.2
  %132 = extractvalue { i1, { i1, i64, i1 } } %131, 1
  %133 = getelementptr inbounds i8, i8* %111, i64 72
  %134 = bitcast i8* %133 to { i1, i64, i1 }*
  store { i1, i64, i1 } %132, { i1, i64, i1 }* %134, align 4
  %135 = getelementptr inbounds i8, i8* %63, i64 128
  %136 = bitcast i8* %135 to { i1, { i1, i64, i1 } }*
  %137 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %136, align 4
  %.fca.0.extract11.i.4 = extractvalue { i1, { i1, i64, i1 } } %137, 0
  br i1 %.fca.0.extract11.i.4, label %__hugr__.const_fun_284.290.exit.4, label %cond_570_case_0.i

__hugr__.const_fun_284.290.exit.4:                ; preds = %__hugr__.const_fun_284.290.exit.3
  %138 = extractvalue { i1, { i1, i64, i1 } } %137, 1
  %139 = getelementptr inbounds i8, i8* %111, i64 96
  %140 = bitcast i8* %139 to { i1, i64, i1 }*
  store { i1, i64, i1 } %138, { i1, i64, i1 }* %140, align 4
  %141 = getelementptr inbounds i8, i8* %63, i64 160
  %142 = bitcast i8* %141 to { i1, { i1, i64, i1 } }*
  %143 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %142, align 4
  %.fca.0.extract11.i.5 = extractvalue { i1, { i1, i64, i1 } } %143, 0
  br i1 %.fca.0.extract11.i.5, label %__hugr__.const_fun_284.290.exit.5, label %cond_570_case_0.i

__hugr__.const_fun_284.290.exit.5:                ; preds = %__hugr__.const_fun_284.290.exit.4
  %144 = extractvalue { i1, { i1, i64, i1 } } %143, 1
  %145 = getelementptr inbounds i8, i8* %111, i64 120
  %146 = bitcast i8* %145 to { i1, i64, i1 }*
  store { i1, i64, i1 } %144, { i1, i64, i1 }* %146, align 4
  %147 = getelementptr inbounds i8, i8* %63, i64 192
  %148 = bitcast i8* %147 to { i1, { i1, i64, i1 } }*
  %149 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %148, align 4
  %.fca.0.extract11.i.6 = extractvalue { i1, { i1, i64, i1 } } %149, 0
  br i1 %.fca.0.extract11.i.6, label %__hugr__.const_fun_284.290.exit.6, label %cond_570_case_0.i

__hugr__.const_fun_284.290.exit.6:                ; preds = %__hugr__.const_fun_284.290.exit.5
  %150 = extractvalue { i1, { i1, i64, i1 } } %149, 1
  %151 = getelementptr inbounds i8, i8* %111, i64 144
  %152 = bitcast i8* %151 to { i1, i64, i1 }*
  store { i1, i64, i1 } %150, { i1, i64, i1 }* %152, align 4
  %153 = getelementptr inbounds i8, i8* %63, i64 224
  %154 = bitcast i8* %153 to { i1, { i1, i64, i1 } }*
  %155 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %154, align 4
  %.fca.0.extract11.i.7 = extractvalue { i1, { i1, i64, i1 } } %155, 0
  br i1 %.fca.0.extract11.i.7, label %__hugr__.const_fun_284.290.exit.7, label %cond_570_case_0.i

__hugr__.const_fun_284.290.exit.7:                ; preds = %__hugr__.const_fun_284.290.exit.6
  %156 = extractvalue { i1, { i1, i64, i1 } } %155, 1
  %157 = getelementptr inbounds i8, i8* %111, i64 168
  %158 = bitcast i8* %157 to { i1, i64, i1 }*
  store { i1, i64, i1 } %156, { i1, i64, i1 }* %158, align 4
  %159 = getelementptr inbounds i8, i8* %63, i64 256
  %160 = bitcast i8* %159 to { i1, { i1, i64, i1 } }*
  %161 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %160, align 4
  %.fca.0.extract11.i.8 = extractvalue { i1, { i1, i64, i1 } } %161, 0
  br i1 %.fca.0.extract11.i.8, label %__hugr__.const_fun_284.290.exit.8, label %cond_570_case_0.i

__hugr__.const_fun_284.290.exit.8:                ; preds = %__hugr__.const_fun_284.290.exit.7
  %162 = extractvalue { i1, { i1, i64, i1 } } %161, 1
  %163 = getelementptr inbounds i8, i8* %111, i64 192
  %164 = bitcast i8* %163 to { i1, i64, i1 }*
  store { i1, i64, i1 } %162, { i1, i64, i1 }* %164, align 4
  %165 = getelementptr inbounds i8, i8* %63, i64 288
  %166 = bitcast i8* %165 to { i1, { i1, i64, i1 } }*
  %167 = load { i1, { i1, i64, i1 } }, { i1, { i1, i64, i1 } }* %166, align 4
  %.fca.0.extract11.i.9 = extractvalue { i1, { i1, i64, i1 } } %167, 0
  br i1 %.fca.0.extract11.i.9, label %__hugr__.const_fun_284.290.exit.9, label %cond_570_case_0.i

__hugr__.const_fun_284.290.exit.9:                ; preds = %__hugr__.const_fun_284.290.exit.8
  %168 = extractvalue { i1, { i1, i64, i1 } } %167, 1
  %169 = getelementptr inbounds i8, i8* %111, i64 216
  %170 = bitcast i8* %169 to { i1, i64, i1 }*
  store { i1, i64, i1 } %168, { i1, i64, i1 }* %170, align 4
  tail call void @heap_free(i8* nonnull %63)
  tail call void @heap_free(i8* nonnull %64)
  br label %__barray_check_bounds.exit888

cond_165_case_0:                                  ; preds = %cond_exit_165
  %171 = load i64, i64* %114, align 4
  %172 = or i64 %171, -1024
  store i64 %172, i64* %114, align 4
  %173 = icmp eq i64 %172, -1
  br i1 %173, label %loop_out139, label %mask_block_err.i886

mask_block_err.i886:                              ; preds = %cond_165_case_0
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([70 x i8], [70 x i8]* @"e_Array cont.EFA5AC45.0", i64 0, i64 0))
  unreachable

__barray_check_bounds.exit888:                    ; preds = %__hugr__.const_fun_284.290.exit.9, %cond_exit_165
  %"167_0.0990" = phi i64 [ 0, %__hugr__.const_fun_284.290.exit.9 ], [ %174, %cond_exit_165 ]
  %174 = add nuw nsw i64 %"167_0.0990", 1
  %175 = lshr i64 %"167_0.0990", 6
  %176 = getelementptr inbounds i64, i64* %114, i64 %175
  %177 = load i64, i64* %176, align 4
  %178 = shl nuw nsw i64 1, %"167_0.0990"
  %179 = and i64 %177, %178
  %.not = icmp eq i64 %179, 0
  br i1 %.not, label %__barray_mask_borrow.exit893, label %cond_exit_165

__barray_mask_borrow.exit893:                     ; preds = %__barray_check_bounds.exit888
  %180 = xor i64 %177, %178
  store i64 %180, i64* %176, align 4
  %181 = getelementptr inbounds { i1, i64, i1 }, { i1, i64, i1 }* %112, i64 %"167_0.0990"
  %182 = load { i1, i64, i1 }, { i1, i64, i1 }* %181, align 4
  %.fca.0.extract511 = extractvalue { i1, i64, i1 } %182, 0
  br i1 %.fca.0.extract511, label %cond_501_case_1, label %cond_exit_165

cond_exit_165:                                    ; preds = %cond_501_case_1, %__barray_mask_borrow.exit893, %__barray_check_bounds.exit888
  %183 = icmp ult i64 %"167_0.0990", 9
  br i1 %183, label %__barray_check_bounds.exit888, label %cond_165_case_0

loop_out139:                                      ; preds = %cond_165_case_0
  tail call void @heap_free(i8* %111)
  tail call void @heap_free(i8* nonnull %113)
  %184 = load i64, i64* %87, align 4
  %185 = and i64 %184, 1023
  store i64 %185, i64* %87, align 4
  %186 = icmp eq i64 %185, 0
  br i1 %186, label %__barray_check_none_borrowed.exit898, label %mask_block_err.i897

__barray_check_none_borrowed.exit898:             ; preds = %loop_out139
  %187 = tail call i8* @heap_alloc(i64 10)
  %188 = tail call i8* @heap_alloc(i64 8)
  %189 = bitcast i8* %188 to i64*
  store i64 0, i64* %189, align 1
  %190 = load { i1, i64, i1 }, { i1, i64, i1 }* %85, align 4
  %.fca.0.extract.i899 = extractvalue { i1, i64, i1 } %190, 0
  %.fca.1.extract.i900 = extractvalue { i1, i64, i1 } %190, 1
  br i1 %.fca.0.extract.i899, label %cond_300_case_1.i, label %cond_300_case_0.i

mask_block_err.i897:                              ; preds = %loop_out139
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([48 x i8], [48 x i8]* @"e_Some array.A77EF32E.0", i64 0, i64 0))
  unreachable

cond_501_case_1:                                  ; preds = %__barray_mask_borrow.exit893
  %.fca.1.extract512 = extractvalue { i1, i64, i1 } %182, 1
  tail call void @___dec_future_refcount(i64 %.fca.1.extract512)
  br label %cond_exit_165

cond_300_case_0.i:                                ; preds = %__barray_check_none_borrowed.exit898
  %.fca.2.extract.i = extractvalue { i1, i64, i1 } %190, 2
  br label %__hugr__.array.__read_bool.3.271.exit

cond_300_case_1.i:                                ; preds = %__barray_check_none_borrowed.exit898
  %read_bool.i = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900)
  br label %__hugr__.array.__read_bool.3.271.exit

__hugr__.array.__read_bool.3.271.exit:            ; preds = %cond_300_case_0.i, %cond_300_case_1.i
  %"03.0.i" = phi i1 [ %read_bool.i, %cond_300_case_1.i ], [ %.fca.2.extract.i, %cond_300_case_0.i ]
  %191 = bitcast i8* %187 to i1*
  store i1 %"03.0.i", i1* %191, align 1
  %192 = getelementptr inbounds i8, i8* %84, i64 24
  %193 = bitcast i8* %192 to { i1, i64, i1 }*
  %194 = load { i1, i64, i1 }, { i1, i64, i1 }* %193, align 4
  %.fca.0.extract.i899.1 = extractvalue { i1, i64, i1 } %194, 0
  %.fca.1.extract.i900.1 = extractvalue { i1, i64, i1 } %194, 1
  br i1 %.fca.0.extract.i899.1, label %cond_300_case_1.i.1, label %cond_300_case_0.i.1

cond_300_case_0.i.1:                              ; preds = %__hugr__.array.__read_bool.3.271.exit
  %.fca.2.extract.i.1 = extractvalue { i1, i64, i1 } %194, 2
  br label %__hugr__.array.__read_bool.3.271.exit.1

cond_300_case_1.i.1:                              ; preds = %__hugr__.array.__read_bool.3.271.exit
  %read_bool.i.1 = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900.1)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900.1)
  br label %__hugr__.array.__read_bool.3.271.exit.1

__hugr__.array.__read_bool.3.271.exit.1:          ; preds = %cond_300_case_1.i.1, %cond_300_case_0.i.1
  %"03.0.i.1" = phi i1 [ %read_bool.i.1, %cond_300_case_1.i.1 ], [ %.fca.2.extract.i.1, %cond_300_case_0.i.1 ]
  %195 = getelementptr inbounds i8, i8* %187, i64 1
  %196 = bitcast i8* %195 to i1*
  store i1 %"03.0.i.1", i1* %196, align 1
  %197 = getelementptr inbounds i8, i8* %84, i64 48
  %198 = bitcast i8* %197 to { i1, i64, i1 }*
  %199 = load { i1, i64, i1 }, { i1, i64, i1 }* %198, align 4
  %.fca.0.extract.i899.2 = extractvalue { i1, i64, i1 } %199, 0
  %.fca.1.extract.i900.2 = extractvalue { i1, i64, i1 } %199, 1
  br i1 %.fca.0.extract.i899.2, label %cond_300_case_1.i.2, label %cond_300_case_0.i.2

cond_300_case_0.i.2:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.1
  %.fca.2.extract.i.2 = extractvalue { i1, i64, i1 } %199, 2
  br label %__hugr__.array.__read_bool.3.271.exit.2

cond_300_case_1.i.2:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.1
  %read_bool.i.2 = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900.2)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900.2)
  br label %__hugr__.array.__read_bool.3.271.exit.2

__hugr__.array.__read_bool.3.271.exit.2:          ; preds = %cond_300_case_1.i.2, %cond_300_case_0.i.2
  %"03.0.i.2" = phi i1 [ %read_bool.i.2, %cond_300_case_1.i.2 ], [ %.fca.2.extract.i.2, %cond_300_case_0.i.2 ]
  %200 = getelementptr inbounds i8, i8* %187, i64 2
  %201 = bitcast i8* %200 to i1*
  store i1 %"03.0.i.2", i1* %201, align 1
  %202 = getelementptr inbounds i8, i8* %84, i64 72
  %203 = bitcast i8* %202 to { i1, i64, i1 }*
  %204 = load { i1, i64, i1 }, { i1, i64, i1 }* %203, align 4
  %.fca.0.extract.i899.3 = extractvalue { i1, i64, i1 } %204, 0
  %.fca.1.extract.i900.3 = extractvalue { i1, i64, i1 } %204, 1
  br i1 %.fca.0.extract.i899.3, label %cond_300_case_1.i.3, label %cond_300_case_0.i.3

cond_300_case_0.i.3:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.2
  %.fca.2.extract.i.3 = extractvalue { i1, i64, i1 } %204, 2
  br label %__hugr__.array.__read_bool.3.271.exit.3

cond_300_case_1.i.3:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.2
  %read_bool.i.3 = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900.3)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900.3)
  br label %__hugr__.array.__read_bool.3.271.exit.3

__hugr__.array.__read_bool.3.271.exit.3:          ; preds = %cond_300_case_1.i.3, %cond_300_case_0.i.3
  %"03.0.i.3" = phi i1 [ %read_bool.i.3, %cond_300_case_1.i.3 ], [ %.fca.2.extract.i.3, %cond_300_case_0.i.3 ]
  %205 = getelementptr inbounds i8, i8* %187, i64 3
  %206 = bitcast i8* %205 to i1*
  store i1 %"03.0.i.3", i1* %206, align 1
  %207 = getelementptr inbounds i8, i8* %84, i64 96
  %208 = bitcast i8* %207 to { i1, i64, i1 }*
  %209 = load { i1, i64, i1 }, { i1, i64, i1 }* %208, align 4
  %.fca.0.extract.i899.4 = extractvalue { i1, i64, i1 } %209, 0
  %.fca.1.extract.i900.4 = extractvalue { i1, i64, i1 } %209, 1
  br i1 %.fca.0.extract.i899.4, label %cond_300_case_1.i.4, label %cond_300_case_0.i.4

cond_300_case_0.i.4:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.3
  %.fca.2.extract.i.4 = extractvalue { i1, i64, i1 } %209, 2
  br label %__hugr__.array.__read_bool.3.271.exit.4

cond_300_case_1.i.4:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.3
  %read_bool.i.4 = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900.4)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900.4)
  br label %__hugr__.array.__read_bool.3.271.exit.4

__hugr__.array.__read_bool.3.271.exit.4:          ; preds = %cond_300_case_1.i.4, %cond_300_case_0.i.4
  %"03.0.i.4" = phi i1 [ %read_bool.i.4, %cond_300_case_1.i.4 ], [ %.fca.2.extract.i.4, %cond_300_case_0.i.4 ]
  %210 = getelementptr inbounds i8, i8* %187, i64 4
  %211 = bitcast i8* %210 to i1*
  store i1 %"03.0.i.4", i1* %211, align 1
  %212 = getelementptr inbounds i8, i8* %84, i64 120
  %213 = bitcast i8* %212 to { i1, i64, i1 }*
  %214 = load { i1, i64, i1 }, { i1, i64, i1 }* %213, align 4
  %.fca.0.extract.i899.5 = extractvalue { i1, i64, i1 } %214, 0
  %.fca.1.extract.i900.5 = extractvalue { i1, i64, i1 } %214, 1
  br i1 %.fca.0.extract.i899.5, label %cond_300_case_1.i.5, label %cond_300_case_0.i.5

cond_300_case_0.i.5:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.4
  %.fca.2.extract.i.5 = extractvalue { i1, i64, i1 } %214, 2
  br label %__hugr__.array.__read_bool.3.271.exit.5

cond_300_case_1.i.5:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.4
  %read_bool.i.5 = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900.5)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900.5)
  br label %__hugr__.array.__read_bool.3.271.exit.5

__hugr__.array.__read_bool.3.271.exit.5:          ; preds = %cond_300_case_1.i.5, %cond_300_case_0.i.5
  %"03.0.i.5" = phi i1 [ %read_bool.i.5, %cond_300_case_1.i.5 ], [ %.fca.2.extract.i.5, %cond_300_case_0.i.5 ]
  %215 = getelementptr inbounds i8, i8* %187, i64 5
  %216 = bitcast i8* %215 to i1*
  store i1 %"03.0.i.5", i1* %216, align 1
  %217 = getelementptr inbounds i8, i8* %84, i64 144
  %218 = bitcast i8* %217 to { i1, i64, i1 }*
  %219 = load { i1, i64, i1 }, { i1, i64, i1 }* %218, align 4
  %.fca.0.extract.i899.6 = extractvalue { i1, i64, i1 } %219, 0
  %.fca.1.extract.i900.6 = extractvalue { i1, i64, i1 } %219, 1
  br i1 %.fca.0.extract.i899.6, label %cond_300_case_1.i.6, label %cond_300_case_0.i.6

cond_300_case_0.i.6:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.5
  %.fca.2.extract.i.6 = extractvalue { i1, i64, i1 } %219, 2
  br label %__hugr__.array.__read_bool.3.271.exit.6

cond_300_case_1.i.6:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.5
  %read_bool.i.6 = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900.6)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900.6)
  br label %__hugr__.array.__read_bool.3.271.exit.6

__hugr__.array.__read_bool.3.271.exit.6:          ; preds = %cond_300_case_1.i.6, %cond_300_case_0.i.6
  %"03.0.i.6" = phi i1 [ %read_bool.i.6, %cond_300_case_1.i.6 ], [ %.fca.2.extract.i.6, %cond_300_case_0.i.6 ]
  %220 = getelementptr inbounds i8, i8* %187, i64 6
  %221 = bitcast i8* %220 to i1*
  store i1 %"03.0.i.6", i1* %221, align 1
  %222 = getelementptr inbounds i8, i8* %84, i64 168
  %223 = bitcast i8* %222 to { i1, i64, i1 }*
  %224 = load { i1, i64, i1 }, { i1, i64, i1 }* %223, align 4
  %.fca.0.extract.i899.7 = extractvalue { i1, i64, i1 } %224, 0
  %.fca.1.extract.i900.7 = extractvalue { i1, i64, i1 } %224, 1
  br i1 %.fca.0.extract.i899.7, label %cond_300_case_1.i.7, label %cond_300_case_0.i.7

cond_300_case_0.i.7:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.6
  %.fca.2.extract.i.7 = extractvalue { i1, i64, i1 } %224, 2
  br label %__hugr__.array.__read_bool.3.271.exit.7

cond_300_case_1.i.7:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.6
  %read_bool.i.7 = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900.7)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900.7)
  br label %__hugr__.array.__read_bool.3.271.exit.7

__hugr__.array.__read_bool.3.271.exit.7:          ; preds = %cond_300_case_1.i.7, %cond_300_case_0.i.7
  %"03.0.i.7" = phi i1 [ %read_bool.i.7, %cond_300_case_1.i.7 ], [ %.fca.2.extract.i.7, %cond_300_case_0.i.7 ]
  %225 = getelementptr inbounds i8, i8* %187, i64 7
  %226 = bitcast i8* %225 to i1*
  store i1 %"03.0.i.7", i1* %226, align 1
  %227 = getelementptr inbounds i8, i8* %84, i64 192
  %228 = bitcast i8* %227 to { i1, i64, i1 }*
  %229 = load { i1, i64, i1 }, { i1, i64, i1 }* %228, align 4
  %.fca.0.extract.i899.8 = extractvalue { i1, i64, i1 } %229, 0
  %.fca.1.extract.i900.8 = extractvalue { i1, i64, i1 } %229, 1
  br i1 %.fca.0.extract.i899.8, label %cond_300_case_1.i.8, label %cond_300_case_0.i.8

cond_300_case_0.i.8:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.7
  %.fca.2.extract.i.8 = extractvalue { i1, i64, i1 } %229, 2
  br label %__hugr__.array.__read_bool.3.271.exit.8

cond_300_case_1.i.8:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.7
  %read_bool.i.8 = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900.8)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900.8)
  br label %__hugr__.array.__read_bool.3.271.exit.8

__hugr__.array.__read_bool.3.271.exit.8:          ; preds = %cond_300_case_1.i.8, %cond_300_case_0.i.8
  %"03.0.i.8" = phi i1 [ %read_bool.i.8, %cond_300_case_1.i.8 ], [ %.fca.2.extract.i.8, %cond_300_case_0.i.8 ]
  %230 = getelementptr inbounds i8, i8* %187, i64 8
  %231 = bitcast i8* %230 to i1*
  store i1 %"03.0.i.8", i1* %231, align 1
  %232 = getelementptr inbounds i8, i8* %84, i64 216
  %233 = bitcast i8* %232 to { i1, i64, i1 }*
  %234 = load { i1, i64, i1 }, { i1, i64, i1 }* %233, align 4
  %.fca.0.extract.i899.9 = extractvalue { i1, i64, i1 } %234, 0
  %.fca.1.extract.i900.9 = extractvalue { i1, i64, i1 } %234, 1
  br i1 %.fca.0.extract.i899.9, label %cond_300_case_1.i.9, label %cond_300_case_0.i.9

cond_300_case_0.i.9:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.8
  %.fca.2.extract.i.9 = extractvalue { i1, i64, i1 } %234, 2
  br label %__hugr__.array.__read_bool.3.271.exit.9

cond_300_case_1.i.9:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.8
  %read_bool.i.9 = tail call i1 @___read_future_bool(i64 %.fca.1.extract.i900.9)
  tail call void @___dec_future_refcount(i64 %.fca.1.extract.i900.9)
  br label %__hugr__.array.__read_bool.3.271.exit.9

__hugr__.array.__read_bool.3.271.exit.9:          ; preds = %cond_300_case_1.i.9, %cond_300_case_0.i.9
  %"03.0.i.9" = phi i1 [ %read_bool.i.9, %cond_300_case_1.i.9 ], [ %.fca.2.extract.i.9, %cond_300_case_0.i.9 ]
  %235 = getelementptr inbounds i8, i8* %187, i64 9
  %236 = bitcast i8* %235 to i1*
  store i1 %"03.0.i.9", i1* %236, align 1
  tail call void @heap_free(i8* nonnull %84)
  tail call void @heap_free(i8* nonnull %86)
  %237 = load i64, i64* %189, align 4
  %238 = and i64 %237, 1023
  store i64 %238, i64* %189, align 4
  %239 = icmp eq i64 %238, 0
  br i1 %239, label %__barray_check_none_borrowed.exit905, label %mask_block_err.i904

__barray_check_none_borrowed.exit905:             ; preds = %__hugr__.array.__read_bool.3.271.exit.9
  %out_arr_alloca = alloca <{ i32, i32, i1*, i1* }>, align 8
  %x_ptr = getelementptr inbounds <{ i32, i32, i1*, i1* }>, <{ i32, i32, i1*, i1* }>* %out_arr_alloca, i64 0, i32 0
  %y_ptr = getelementptr inbounds <{ i32, i32, i1*, i1* }>, <{ i32, i32, i1*, i1* }>* %out_arr_alloca, i64 0, i32 1
  %arr_ptr = getelementptr inbounds <{ i32, i32, i1*, i1* }>, <{ i32, i32, i1*, i1* }>* %out_arr_alloca, i64 0, i32 2
  %mask_ptr = getelementptr inbounds <{ i32, i32, i1*, i1* }>, <{ i32, i32, i1*, i1* }>* %out_arr_alloca, i64 0, i32 3
  %240 = alloca [10 x i1], align 1
  %.sub = getelementptr inbounds [10 x i1], [10 x i1]* %240, i64 0, i64 0
  %241 = bitcast [10 x i1]* %240 to i8*
  call void @llvm.memset.p0i8.i64(i8* noundef nonnull align 1 dereferenceable(10) %241, i8 0, i64 10, i1 false)
  store i32 10, i32* %x_ptr, align 8
  store i32 1, i32* %y_ptr, align 4
  %242 = bitcast i1** %arr_ptr to i8**
  store i8* %187, i8** %242, align 8
  store i1* %.sub, i1** %mask_ptr, align 8
  call void @print_bool_arr(i8* getelementptr inbounds ([16 x i8], [16 x i8]* @res_cs.46C3C4B5.0, i64 0, i64 0), i64 15, <{ i32, i32, i1*, i1* }>* nonnull %out_arr_alloca)
  br label %__barray_check_bounds.exit913

mask_block_err.i904:                              ; preds = %__hugr__.array.__read_bool.3.271.exit.9
  tail call void @panic(i32 1002, i8* getelementptr inbounds ([48 x i8], [48 x i8]* @"e_Some array.A77EF32E.0", i64 0, i64 0))
  unreachable

__barray_check_bounds.exit913:                    ; preds = %cond_exit_95, %__barray_check_none_borrowed.exit905
  %"90_0.sroa.0.0972" = phi i64 [ 0, %__barray_check_none_borrowed.exit905 ], [ %243, %cond_exit_95 ]
  %243 = add nuw nsw i64 %"90_0.sroa.0.0972", 1
  %244 = lshr i64 %"90_0.sroa.0.0972", 6
  %245 = getelementptr inbounds i64, i64* %7, i64 %244
  %246 = load i64, i64* %245, align 4
  %247 = and i64 %"90_0.sroa.0.0972", 63
  %248 = shl nuw i64 1, %247
  %249 = and i64 %246, %248
  %.not.i914 = icmp eq i64 %249, 0
  br i1 %.not.i914, label %panic.i915, label %cond_exit_95

panic.i915:                                       ; preds = %__barray_check_bounds.exit913
  call void @panic(i32 1002, i8* getelementptr inbounds ([57 x i8], [57 x i8]* @"e_Array alre.5A300C2A.0", i64 0, i64 0))
  unreachable

cond_exit_95:                                     ; preds = %__barray_check_bounds.exit913
  %250 = xor i64 %246, %248
  store i64 %250, i64* %245, align 4
  %251 = getelementptr inbounds i64, i64* %5, i64 %"90_0.sroa.0.0972"
  store i64 %"90_0.sroa.0.0972", i64* %251, align 4
  %exitcond984.not = icmp eq i64 %243, 100
  br i1 %exitcond984.not, label %loop_out212, label %__barray_check_bounds.exit913

loop_out212:                                      ; preds = %cond_exit_95
  %252 = getelementptr inbounds i8, i8* %6, i64 8
  %253 = bitcast i8* %252 to i64*
  %254 = load i64, i64* %253, align 4
  %255 = and i64 %254, 68719476735
  store i64 %255, i64* %253, align 4
  %256 = load i64, i64* %7, align 4
  %257 = icmp eq i64 %256, 0
  %258 = icmp eq i64 %255, 0
  %or.cond = select i1 %257, i1 %258, i1 false
  br i1 %or.cond, label %__barray_check_none_borrowed.exit921, label %mask_block_err.i920

__barray_check_none_borrowed.exit921:             ; preds = %loop_out212
  %259 = call i8* @heap_alloc(i64 800)
  %260 = bitcast i8* %259 to i64*
  %261 = call i8* @heap_alloc(i64 16)
  %262 = bitcast i8* %261 to i64*
  call void @llvm.memset.p0i64.i64(i64* noundef nonnull align 1 dereferenceable(16) %262, i8 0, i64 16, i1 false)
  call void @llvm.memcpy.p0i64.p0i64.i64(i64* noundef nonnull align 1 dereferenceable(800) %260, i64* noundef nonnull align 1 dereferenceable(800) %5, i64 800, i1 false)
  call void @heap_free(i8* %259)
  %263 = load i64, i64* %253, align 4
  %264 = and i64 %263, 68719476735
  store i64 %264, i64* %253, align 4
  %265 = load i64, i64* %7, align 4
  %266 = icmp eq i64 %265, 0
  %267 = icmp eq i64 %264, 0
  %or.cond987 = select i1 %266, i1 %267, i1 false
  br i1 %or.cond987, label %__barray_check_none_borrowed.exit926, label %mask_block_err.i925

mask_block_err.i920:                              ; preds = %loop_out212
  call void @panic(i32 1002, i8* getelementptr inbounds ([48 x i8], [48 x i8]* @"e_Some array.A77EF32E.0", i64 0, i64 0))
  unreachable

__barray_check_none_borrowed.exit926:             ; preds = %__barray_check_none_borrowed.exit921
  %out_arr_alloca287 = alloca <{ i32, i32, i64*, i1* }>, align 8
  %x_ptr288 = getelementptr inbounds <{ i32, i32, i64*, i1* }>, <{ i32, i32, i64*, i1* }>* %out_arr_alloca287, i64 0, i32 0
  %y_ptr289 = getelementptr inbounds <{ i32, i32, i64*, i1* }>, <{ i32, i32, i64*, i1* }>* %out_arr_alloca287, i64 0, i32 1
  %arr_ptr290 = getelementptr inbounds <{ i32, i32, i64*, i1* }>, <{ i32, i32, i64*, i1* }>* %out_arr_alloca287, i64 0, i32 2
  %mask_ptr291 = getelementptr inbounds <{ i32, i32, i64*, i1* }>, <{ i32, i32, i64*, i1* }>* %out_arr_alloca287, i64 0, i32 3
  %268 = alloca [100 x i1], align 1
  %.sub635 = getelementptr inbounds [100 x i1], [100 x i1]* %268, i64 0, i64 0
  %269 = bitcast [100 x i1]* %268 to i8*
  call void @llvm.memset.p0i8.i64(i8* noundef nonnull align 1 dereferenceable(100) %269, i8 0, i64 100, i1 false)
  store i32 100, i32* %x_ptr288, align 8
  store i32 1, i32* %y_ptr289, align 4
  %270 = bitcast i64** %arr_ptr290 to i8**
  store i8* %4, i8** %270, align 8
  store i1* %.sub635, i1** %mask_ptr291, align 8
  call void @print_int_arr(i8* getelementptr inbounds ([15 x i8], [15 x i8]* @res_is.F21393DB.0, i64 0, i64 0), i64 14, <{ i32, i32, i64*, i1* }>* nonnull %out_arr_alloca287)
  br label %__barray_check_bounds.exit934

mask_block_err.i925:                              ; preds = %__barray_check_none_borrowed.exit921
  call void @panic(i32 1002, i8* getelementptr inbounds ([48 x i8], [48 x i8]* @"e_Some array.A77EF32E.0", i64 0, i64 0))
  unreachable

__barray_check_bounds.exit934:                    ; preds = %cond_exit_130, %__barray_check_none_borrowed.exit926
  %"125_0.sroa.0.0974" = phi i64 [ 0, %__barray_check_none_borrowed.exit926 ], [ %271, %cond_exit_130 ]
  %271 = add nuw nsw i64 %"125_0.sroa.0.0974", 1
  %272 = lshr i64 %"125_0.sroa.0.0974", 6
  %273 = getelementptr inbounds i64, i64* %3, i64 %272
  %274 = load i64, i64* %273, align 4
  %275 = and i64 %"125_0.sroa.0.0974", 63
  %276 = shl nuw i64 1, %275
  %277 = and i64 %274, %276
  %.not.i935 = icmp eq i64 %277, 0
  br i1 %.not.i935, label %panic.i936, label %cond_exit_130

panic.i936:                                       ; preds = %__barray_check_bounds.exit934
  call void @panic(i32 1002, i8* getelementptr inbounds ([57 x i8], [57 x i8]* @"e_Array alre.5A300C2A.0", i64 0, i64 0))
  unreachable

cond_exit_130:                                    ; preds = %__barray_check_bounds.exit934
  %278 = sitofp i64 %"125_0.sroa.0.0974" to double
  %279 = fmul double %278, 6.250000e-02
  %280 = xor i64 %274, %276
  store i64 %280, i64* %273, align 4
  %281 = getelementptr inbounds double, double* %1, i64 %"125_0.sroa.0.0974"
  store double %279, double* %281, align 8
  %exitcond985.not = icmp eq i64 %271, 100
  br i1 %exitcond985.not, label %loop_out299, label %__barray_check_bounds.exit934

loop_out299:                                      ; preds = %cond_exit_130
  %282 = getelementptr inbounds i8, i8* %2, i64 8
  %283 = bitcast i8* %282 to i64*
  %284 = load i64, i64* %283, align 4
  %285 = and i64 %284, 68719476735
  store i64 %285, i64* %283, align 4
  %286 = load i64, i64* %3, align 4
  %287 = icmp eq i64 %286, 0
  %288 = icmp eq i64 %285, 0
  %or.cond988 = select i1 %287, i1 %288, i1 false
  br i1 %or.cond988, label %__barray_check_none_borrowed.exit942, label %mask_block_err.i941

__barray_check_none_borrowed.exit942:             ; preds = %loop_out299
  %289 = call i8* @heap_alloc(i64 800)
  %290 = bitcast i8* %289 to double*
  %291 = call i8* @heap_alloc(i64 16)
  %292 = bitcast i8* %291 to i64*
  call void @llvm.memset.p0i64.i64(i64* noundef nonnull align 1 dereferenceable(16) %292, i8 0, i64 16, i1 false)
  call void @llvm.memcpy.p0f64.p0f64.i64(double* noundef nonnull align 1 dereferenceable(800) %290, double* noundef nonnull align 1 dereferenceable(800) %1, i64 800, i1 false)
  call void @heap_free(i8* %289)
  %293 = load i64, i64* %283, align 4
  %294 = and i64 %293, 68719476735
  store i64 %294, i64* %283, align 4
  %295 = load i64, i64* %3, align 4
  %296 = icmp eq i64 %295, 0
  %297 = icmp eq i64 %294, 0
  %or.cond989 = select i1 %296, i1 %297, i1 false
  br i1 %or.cond989, label %__barray_check_none_borrowed.exit947, label %mask_block_err.i946

mask_block_err.i941:                              ; preds = %loop_out299
  call void @panic(i32 1002, i8* getelementptr inbounds ([48 x i8], [48 x i8]* @"e_Some array.A77EF32E.0", i64 0, i64 0))
  unreachable

__barray_check_none_borrowed.exit947:             ; preds = %__barray_check_none_borrowed.exit942
  %out_arr_alloca377 = alloca <{ i32, i32, double*, i1* }>, align 8
  %x_ptr378 = getelementptr inbounds <{ i32, i32, double*, i1* }>, <{ i32, i32, double*, i1* }>* %out_arr_alloca377, i64 0, i32 0
  %y_ptr379 = getelementptr inbounds <{ i32, i32, double*, i1* }>, <{ i32, i32, double*, i1* }>* %out_arr_alloca377, i64 0, i32 1
  %arr_ptr380 = getelementptr inbounds <{ i32, i32, double*, i1* }>, <{ i32, i32, double*, i1* }>* %out_arr_alloca377, i64 0, i32 2
  %mask_ptr381 = getelementptr inbounds <{ i32, i32, double*, i1* }>, <{ i32, i32, double*, i1* }>* %out_arr_alloca377, i64 0, i32 3
  %298 = alloca [100 x i1], align 1
  %.sub736 = getelementptr inbounds [100 x i1], [100 x i1]* %298, i64 0, i64 0
  %299 = bitcast [100 x i1]* %298 to i8*
  call void @llvm.memset.p0i8.i64(i8* noundef nonnull align 1 dereferenceable(100) %299, i8 0, i64 100, i1 false)
  store i32 100, i32* %x_ptr378, align 8
  store i32 1, i32* %y_ptr379, align 4
  %300 = bitcast double** %arr_ptr380 to i8**
  store i8* %0, i8** %300, align 8
  store i1* %.sub736, i1** %mask_ptr381, align 8
  call void @print_float_arr(i8* getelementptr inbounds ([17 x i8], [17 x i8]* @res_fs.CBD4AF54.0, i64 0, i64 0), i64 16, <{ i32, i32, double*, i1* }>* nonnull %out_arr_alloca377)
  ret void

mask_block_err.i946:                              ; preds = %__barray_check_none_borrowed.exit942
  call void @panic(i32 1002, i8* getelementptr inbounds ([48 x i8], [48 x i8]* @"e_Some array.A77EF32E.0", i64 0, i64 0))
  unreachable
}

declare i8* @heap_alloc(i64) local_unnamed_addr

; Function Attrs: argmemonly mustprogress nofree nounwind willreturn writeonly
declare void @llvm.memset.p0i64.i64(i64* nocapture writeonly, i8, i64, i1 immarg) #0

; Function Attrs: noreturn
declare void @panic(i32, i8*) local_unnamed_addr #1

declare void @heap_free(i8*) local_unnamed_addr

declare void @___dec_future_refcount(i64) local_unnamed_addr

declare void @print_bool_arr(i8*, i64, <{ i32, i32, i1*, i1* }>*) local_unnamed_addr

; Function Attrs: argmemonly mustprogress nofree nounwind willreturn
declare void @llvm.memcpy.p0i64.p0i64.i64(i64* noalias nocapture writeonly, i64* noalias nocapture readonly, i64, i1 immarg) #2

declare void @print_int_arr(i8*, i64, <{ i32, i32, i64*, i1* }>*) local_unnamed_addr

; Function Attrs: argmemonly mustprogress nofree nounwind willreturn
declare void @llvm.memcpy.p0f64.p0f64.i64(double* noalias nocapture writeonly, double* noalias nocapture readonly, i64, i1 immarg) #2

declare void @print_float_arr(i8*, i64, <{ i32, i32, double*, i1* }>*) local_unnamed_addr

declare i1 @___read_future_bool(i64) local_unnamed_addr

declare i64 @___lazy_measure(i64) local_unnamed_addr

declare void @___qfree(i64) local_unnamed_addr

declare i64 @___qalloc() local_unnamed_addr

declare void @___reset(i64) local_unnamed_addr

declare void @___rxy(i64, double, double) local_unnamed_addr

declare void @___inc_future_refcount(i64) local_unnamed_addr

define i64 @qmain(i64 %0) local_unnamed_addr {
entry:
  tail call void @setup(i64 %0)
  tail call fastcc void @__hugr__.main.1()
  %1 = tail call i64 @teardown()
  ret i64 %1
}

declare void @setup(i64) local_unnamed_addr

declare i64 @teardown() local_unnamed_addr

; Function Attrs: argmemonly nofree nounwind willreturn writeonly
declare void @llvm.memset.p0i8.i64(i8* nocapture writeonly, i8, i64, i1 immarg) #3

attributes #0 = { argmemonly mustprogress nofree nounwind willreturn writeonly }
attributes #1 = { noreturn }
attributes #2 = { argmemonly mustprogress nofree nounwind willreturn }
attributes #3 = { argmemonly nofree nounwind willreturn writeonly }

!name = !{!0}

!0 = !{!"mainlib"}
