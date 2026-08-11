#pragma once

// 仅用于标注函数的参数方向；展开为空，不改变 ABI 或运行时行为。
// 使用示例：void Calculate(const Input& input, _OUT Result& result);
#ifndef _OUT
#define _OUT
#endif
