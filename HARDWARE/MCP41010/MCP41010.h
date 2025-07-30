#ifndef INC_MCP41010_H_
#define INC_MCP41010_H_

#include "main.h"
void PY_Delay_us_t(uint32_t Delay);

void MCP41010_Init(void);
void MCP41010_SHUTDONW(void);
void MCP41010_Write(uint8_t data);

#endif /* INC_MCP41010_H_ */

