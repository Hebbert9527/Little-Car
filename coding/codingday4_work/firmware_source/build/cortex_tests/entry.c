#include <stdint.h>
volatile uint32_t test_result;
const char *test_file,*test_expression;
__attribute__((noinline,noreturn)) void test_done(void){for(;;)__asm volatile("nop");}
int main(void);
int puts(const char *s){(void)s;return 0;}
int printf(const char *s,...){(void)s;return 0;}
void test_entry(void){test_result=main();test_done();}
void __assert_func(const char *file,int line,const char *func,const char *expr)
{(void)func;test_file=file;test_expression=expr;test_result=line;test_done();}
