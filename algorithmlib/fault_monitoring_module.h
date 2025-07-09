#ifndef _FFMODULE_H
#define _FFMODULE_H
#include <stdint.h>
// 状态返回值定义
#define FMM_NORMAL     0  // 正常状态
#define FMM_WARNING    1  // 超限但未触发故障
#define FMM_FAULT      -1 // 故障已触发
#define FMM_RECOVERING 2  // 故障恢复中
#define FMM_INVALID    -2 // 参数错误

typedef struct fault_monitoring_module {
	uint16_t fault_count;
	uint16_t fault_limit_count;
	uint16_t recovery_count;
	uint16_t recovery_limit_count;
	int16_t fault_status;
	float up_limit_value;
	float down_limit_value;
	void (*cbx)(void); // 使用函数指针类型
} fmm_t;

void fmm_init(fmm_t *fmm, float ulv, float dlv, uint16_t fault_limit_count,
	      uint16_t recovery_limit_count, void (*cbx)(void));
void fmm_reset(fmm_t *fmm);
int fmm_monitoring(fmm_t *fmm, float cur_value);

#endif
