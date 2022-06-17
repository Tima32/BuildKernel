#ifndef GRIF_FPGA_MGR_BENCH_H_
#define GRIF_FPGA_MGR_BENCH_H_

struct througput_test_settings {
	struct device *dev;

	void __iomem *base;
	int mem_offset;
	size_t mem_size;

	// Number of times averaging will be made
	int bench_repeats;

	// Number of cycles that used in single adress test
	int cycles;

	// TODO: Not implemented
	enum {
		TEST_MODE_READ_ONLY,
		TEST_MODE_WRITE_ONLY,
		TEST_MODE_RW_INTERLEAVE,
		TEST_MODE_RW_RANDOM,
		TEST_MODE_ALL,
	} rw_mode;
	enum {
		TEST_ACCESS_SEQUENTIAL,
		TEST_ACCESS_RANDOM,
		TEST_ACCESS_ALL,
	} access_mode;
	enum {
		TEST_IO_8b = 1,
		TEST_IO_16b = 2,
		TEST_IO_32b = 4,
		TEST_IO_ALL
	} io_size;

	// Private data for time calculations corrections
	ktime_t offset_correction;
	ktime_t cycle_offset_correction;
	ktime_t cycle_offset;
};

void grif_througput_test(struct througput_test_settings *s);

#endif /* GRIF_FPGA_MGR_BENCH_H_ */
