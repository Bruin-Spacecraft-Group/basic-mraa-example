#include<mraa.h>
#include<stdio.h>


mraa_gpio_context gp;

int main(int argc, char** argv) {


	fprintf(stdout, "argc: %d\n", argc);
	int i = 0;
	for (i = 0; i < argc; i++)
		fprintf(stdout, "%dth argument is %s\n", i+1, argv[i]);


	gp = mraa_gpio_init(7);
	if (!gp) {
		fprintf(stderr, "gpio init on 7 failed with return code\n");
		return 1;
	}

	int gp_state = mraa_gpio_read(gp);
	if (gp_state == -1) {
		fprintf(stderr, "gpio read failed.\n");
		return 1;
	}

	fprintf(stdout, "gpio 4 current value: %d\n", gp_state);

	mraa_gpio_close(gp);

	return 0;
}
