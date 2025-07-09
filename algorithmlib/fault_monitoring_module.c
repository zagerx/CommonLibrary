#include "fault_monitoring_module.h"

void fmm_init(fmm_t *fmm, float ulv, float dlv, uint16_t fault_limit_count,
	      uint16_t recovery_limit_count, void (*cbx)(void))
{
	if (fmm) {
		fmm->down_limit_value = dlv;
		fmm->up_limit_value = ulv;
		fmm->cbx = cbx;
		fmm->fault_limit_count = fault_limit_count;
		fmm->recovery_limit_count = recovery_limit_count;
		fmm->recovery_count = 0;
		fmm->fault_count = 0;
		fmm->fault_status = 0;
	}
}

void fmm_reset(fmm_t *fmm)
{
	if (fmm) {
		fmm->fault_count = 0;
		fmm->fault_status = 0;
		fmm->recovery_count = 0;
	}
}

int fmm_monitoring(fmm_t *fmm, float cur_value)
{
	if (!fmm) {
		return FMM_INVALID;
	}

	if (fmm->fault_status) {
		// 故障状态处理
		if (fmm->down_limit_value < cur_value && cur_value < fmm->up_limit_value) {
			if (++fmm->recovery_count >= fmm->recovery_limit_count) {
				fmm->fault_status = 0;
				fmm->recovery_count = 0; // 仅重置恢复计数
				return FMM_NORMAL;	 // 恢复完成
			}
			return FMM_RECOVERING; // 恢复中
		} else {
			fmm->recovery_count = 0; // 中断恢复
			return FMM_FAULT;
		}
	} else {
		// 正常状态处理
		if (cur_value < fmm->down_limit_value || cur_value > fmm->up_limit_value) {
			if (++fmm->fault_count >= fmm->fault_limit_count) {
				fmm->fault_status = 1;
				fmm->fault_count = 0; // 重置故障计数
				fmm->recovery_count = 0;
				if (fmm->cbx) {
					fmm->cbx();
				}
				return FMM_FAULT;
			}
			return FMM_WARNING; // 超限警告
		} else {
			fmm->fault_count = 0; // 重置故障计数
			return FMM_NORMAL;
		}
	}
}