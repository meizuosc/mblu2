#include <stdio.h>

char *ksyms_name[]= {
	"__init_begin",
	"_sinittext",
	"_einittext",
	"_stext_l1",
	"_etext_l1",
	"_stext_l2",
	"_etext_l2",
	"_stext",
	"_etext",
	"stext",
	"_text",
	"sys_call_table",
	"sys_delete_module",
	"sys_init_module",
	NULL,
};

int cmp_ksym(char *sym)
{
	int find;
	int i;

	for (i = 0, find = 0; ksyms_name[i]; i++) {
		if (!strcmp(ksyms_name[i], sym)) {
			find = 1;
			break;
		}
	}

	return find;
}

void read_ksym(FILE* in)
{
	char str[500];
	unsigned long long addr;
	char stype;
	int rc;

	while (!feof(in)) {
		rc = fscanf(in, "%llx %c %499s\n", &addr, &stype, str);
		if (rc != 3)
			return;
		if (cmp_ksym(str))
			printf("%llx %c %s\n", addr, stype, str);
	}
}

int main (int args, char *argv[])
{
	read_ksym(stdin);
	return 0;
}
