################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
main.obj: ../main.c $(GEN_OPTS) $(GEN_SRCS)
	@echo 'Building file: $<'
	@echo 'Invoking: MSP430 Compiler'
	"F:/TI/ccsv5/tools/compiler/msp430/bin/cl430" -vmsp --abi=coffabi -g --include_path="F:/TI/ccsv5/ccs_base/msp430/include" --include_path="F:/TI/ccsv5/tools/compiler/msp430/include" --define=__MSP430F2272__ --diag_warning=225 --display_error_number --printf_support=minimal --preproc_with_compile --preproc_dependency="main.pp" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '


