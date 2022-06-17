#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/platform_device.h>

#include <asm/io.h>

#include "bench.h"

#define TEST_BENCH(x) ({ \
	ktime_t before, after; \
	before = ktime_get(); \
	{ x; }; \
	after = ktime_get(); \
	ktime_sub(after, before); \
})

#define TEST_BENCH_AVERAGE(x, _repeats, _offset) ({ \
	ktime_t _time; \
	int _repeat; \
	for (_repeat = 0, _time = 0; _repeat < _repeats; _repeat++) { \
		_time += TEST_BENCH(x) - _offset; \
	} \
	div_u64(_time, _repeats); \
})

#define TEST_BENCH_MINIMUM(x, _repeats) ({ \
	ktime_t _time = 0, _min = 0; \
	int _repeat; \
	for (_repeat = 0; _repeat < _repeats; _repeat++) { \
		_time = TEST_BENCH(x); \
		_min = min(_min, _time); \
	} \
	_time; \
})

#define TEST_BENCH_AVG_CYCLED(_func, _bits, ...) ({ \
	ktime_t bench_time = TEST_BENCH_AVERAGE( \
	int cycles; \
	for (cycles = 0; cycles < s->cycles; ++cycles) \
		_func(__VA_ARGS__); \
	, s->bench_repeats, s->cycle_offset_correction); \
	dev_info(s->dev, "GRIF_BENCH: "#_func", %d cycles: %lld nanoseconds (~ %lld Mbit/s)", \
		s->cycles, div_u64(bench_time, s->cycles), div_u64(s->cycles * _bits, div_u64(bench_time, 1000))); \
})

#define TEST_BENCH_WRITE_FROM_MEM(_func, _bits, _from, _to, _size) ({ \
	ktime_t bench_time = TEST_BENCH( \
		int data_idx; \
		for (data_idx = 0; data_idx < _size / (_bits / 8); data_idx++) \
			_func(((u##_bits *)_from)[data_idx], ((u##_bits *)_to) + data_idx); \
	); \
	dev_info(s->dev, "GRIF_BENCH: "#_func" %d bytes from mem: %lld nanoseconds (~ %lld Mbit/s)", \
		_size, bench_time, div_u64(_size * 8, div_u64(bench_time, 1000))); \
})

#define TEST_BENCH_READ_TO_MEM(_func, _bits, _from, _to, _size) ({ \
	ktime_t bench_time = TEST_BENCH( \
		int data_idx; \
		for (data_idx = 0; data_idx < _size / (_bits / 8); data_idx++) \
			((u##_bits *)_to)[data_idx] = _func(((u##_bits *)_from) + data_idx); \
	); \
	dev_info(s->dev, "GRIF_BENCH: "#_func" %d bytes to mem: %lld nanoseconds (~ %lld Mbit/s)", \
		_size, bench_time, div_u64(_size * 8, div_u64(bench_time, 1000))); \
})

void grif_througput_test(struct througput_test_settings *s) {
	unsigned long flags;
	int i;
	void __iomem *addr = s->base + s->mem_offset;
	size_t size = s->mem_size;
	void *rand = devm_kzalloc(s->dev, size, GFP_KERNEL);
	void *buf = devm_kzalloc(s->dev, size, GFP_KERNEL);

	/*******************************
	 * read/write consistency test *
	 *******************************/
	get_random_bytes(rand, size);
	memset(buf, 0xFF, size);
	memset_io(addr, 0, size);

	memcpy_toio(addr, rand, size);
	// FIXME: BUG: Using readb because of problem with memcpy_fromio()
	// memcpy_fromio(buf, addr, size);
	for (i = 0; i < size; ++i)
		((u8 *)buf)[i] = readb(addr + i);

	if (memcmp(rand, buf, size))
		dev_err(s->dev, "GRIF_BENCH: memory corruption detected!");
	else
		dev_info(s->dev, "GRIF_BENCH: checked %d bytes of memory - OK", size);

	preempt_disable();
	local_irq_save(flags);

	/*************************
	 * Pre-test calculations *
	 *************************/

	dev_info(s->dev, "GRIF_BENCH: ktime timer resolution: %d nanoseconds",
			ktime_get_resolution_ns());

	dev_info(s->dev, "GRIF_BENCH: every measurement will repeated %d times and averaged",
			s->bench_repeats);

	/* Measurement time */
	s->offset_correction = TEST_BENCH_MINIMUM(, s->bench_repeats);
	dev_info(s->dev, "GRIF_BENCH: Single measurement (used as fixed offset, already compensated in next results): %lld nanoseconds",
			s->offset_correction);

	/* NOP cycle time */
	{
		ktime_t nop_bench = TEST_BENCH_AVERAGE(
			int cycles;
			for (cycles = 0; cycles < s->cycles; ++cycles)
				asm volatile ("nop")
		, s->bench_repeats, s->offset_correction);
		s->cycle_offset = div_u64(nop_bench, s->cycles);
	}
	dev_info(s->dev, "GRIF_BENCH: Base cycle (nop instruction, used as fixed offset, already compensated in next results where cycles used), %d cycles: %lld nanoseconds",
			s->cycles, s->cycle_offset);
	s->cycle_offset_correction = s->offset_correction + s->cycle_offset;

	/***********************
	 * Single address test *
	 ***********************/
	dev_info(s->dev, "GRIF_BENCH: Single address test:");

	TEST_BENCH_AVG_CYCLED(readb, 8, addr);
	TEST_BENCH_AVG_CYCLED(readw, 16, addr);
	TEST_BENCH_AVG_CYCLED(readl, 32, addr);

	TEST_BENCH_AVG_CYCLED(readb_relaxed, 8, addr);
	TEST_BENCH_AVG_CYCLED(readw_relaxed, 16, addr);
	TEST_BENCH_AVG_CYCLED(readl_relaxed, 32, addr);

	TEST_BENCH_AVG_CYCLED(writeb, 8, 0xFF, addr);
	TEST_BENCH_AVG_CYCLED(writew, 16, 0xFFFF, addr);
	TEST_BENCH_AVG_CYCLED(writel, 32, 0xFFFFFFFF, addr);

	TEST_BENCH_AVG_CYCLED(writeb_relaxed, 8, 0xFF, addr);
	TEST_BENCH_AVG_CYCLED(writew_relaxed, 16, 0xFFFF, addr);
	TEST_BENCH_AVG_CYCLED(writel_relaxed, 32, 0xFFFFFFFF, addr);

	/*******************************
	 * Contiguous memory area test *
	 *******************************/
	if (size > s->io_size) {
		dev_info(s->dev, "GRIF_BENCH: Contiguous memory area test:");

		TEST_BENCH_WRITE_FROM_MEM(writeb, 8, rand, addr, size);
		TEST_BENCH_READ_TO_MEM(readb, 8, addr, buf, size);
		if (memcmp(rand, buf, size))
			dev_err(s->dev, "GRIF_BENCH: memory corruption detected!");

		TEST_BENCH_WRITE_FROM_MEM(writew, 16, rand, addr, size);
		TEST_BENCH_READ_TO_MEM(readw, 16, addr, buf, size);
		if (memcmp(rand, buf, size))
			dev_err(s->dev, "GRIF_BENCH: memory corruption detected!");

		TEST_BENCH_WRITE_FROM_MEM(writel, 32, rand, addr, size);
		TEST_BENCH_READ_TO_MEM(readl, 32, addr, buf, size);
		if (memcmp(rand, buf, size))
			dev_err(s->dev, "GRIF_BENCH: memory corruption detected!");

		{
			ktime_t bench_time = TEST_BENCH_AVERAGE(
				memcpy_toio(addr, rand, size);
			, s->bench_repeats, s->offset_correction);
			dev_info(s->dev, "GRIF_BENCH: memcpy_toio, %d bytes: %lld nanoseconds (~ %lld Mbit/s)",
				size, bench_time, div_u64(size * 8, div_u64(bench_time, 1000)));
		}
		{
			ktime_t bench_time = TEST_BENCH_AVERAGE(
				memcpy_fromio(buf, addr, size);
			, s->bench_repeats, s->offset_correction);
			dev_info(s->dev, "GRIF_BENCH: memcpy_fromio, %d bytes: %lld nanoseconds (~ %lld Mbit/s)",
				size, bench_time, div_u64(size * 8, div_u64(bench_time, 1000)));
		}
		if (memcmp(rand, buf, size))
			dev_err(s->dev, "GRIF_BENCH: memory corruption detected!");
	}

	local_irq_restore(flags);
	preempt_enable();

	devm_kfree(s->dev, buf);
	devm_kfree(s->dev, rand);
}
