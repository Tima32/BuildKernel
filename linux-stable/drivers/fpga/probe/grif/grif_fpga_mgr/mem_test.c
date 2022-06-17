#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/ktime.h>
#include <linux/kernel.h>

#include <common_fpga/fpgafeat.h>
#include "grif_fpga.h"

#define PRINT_TIME( dev, max, min, avg, str ) (\
	dev_notice(dev, "%s time nsec: max %lld min %lld avg %lld", str,\
		ktime_to_ns( max ),\
		ktime_to_ns( min ),\
		ktime_to_ns( avg ) ))

#define SET_TIME_PARAM( t, maxt, mint, sumt) ({\
	if( ktime_compare(t, maxt) > 0 )\
		 maxt = t;\
	if( ktime_compare(t, mint) < 0 )\
		mint = t;\
	sumt = ktime_add( t, sumt);})


#define GET_BIT_FROM( val , bit ) ( ( val >> bit ) & 1 )
#define REPEAT_CNT 1000

static uint16_t prbs = 0xABCD;
//  __attribute__((optimize("O0")))

inline void write( void * context, uint16_t reg, uint16_t val){
	regmap_write( context, reg, val );
}

inline uint16_t read( void * context, uint16_t reg){
	int val;
	regmap_read( context, reg, &val );
	return val;
}

uint16_t gen_prbs( uint16_t min, uint16_t max ){
	uint16_t bin_num = max - min;
	uint32_t m;

	prbs = ( prbs << 1 ) | ( GET_BIT_FROM( prbs, 14 ) ^ GET_BIT_FROM( prbs, 13 ) );
	m = prbs * bin_num;
	return ( m >> 16 ) + min;
}

void check_mem( void * context, uint16_t *ref_mem, uint16_t offset, int * error, int size ){
	int      i;
	uint16_t rd_val = 0;

	for( i = 0; i < size; i++ ){
		rd_val = read( context, offset + i );
		if( *(ref_mem + i) != rd_val ){
			*error = *error + 1;
		}
	}
}


void mem_dump( struct device * dev,  void * context,  uint16_t offset, int size ){
	int      i;
	uint16_t rd_val = 0;

	for( i = 0; i < size; i++ ){
		rd_val = read( context, offset + i );
		dev_dbg( dev, "DUMP addr: %i val: %i\n", i, rd_val );
	}
}

void shift_one( void * context, uint16_t *ref_mem, uint16_t offset, uint16_t addr, int *error ){
	int      i;
	uint16_t val;

	for( i = 0; i < 16; i++ ){
	val = 1 << i;
	write( context, addr + offset, val );
	if(val != read( context, addr + offset ))
		*error = *error + 1;
	}
	*( ref_mem + addr ) = val;
}

void set_array_ref( uint16_t *ref_mem, uint16_t * addr, uint16_t *val, int cnt ){
	int i;
	for( i = 0; i < cnt; i++ )
		*( ref_mem + *( addr + i ) ) = *( val + i );
}

void set_array( void * context, uint16_t * addr, uint16_t * val, uint16_t offset, int cnt ){
	int i;
	for( i = 0; i < cnt; i++ ){
		write( context, offset + *( addr + i ), *(val + i ) );
	}
}

void get_array( void * context, uint16_t * addr, uint16_t * val, int *error, int cnt ){
	int i;
	for( i = 0; i < cnt; i++ )
		*(val + i) = read( context, *( addr + i ) );
}

void random( void * context, uint16_t *ref_mem, uint16_t offset, int *error, int mem_size, int cnt ){
	int      i;
	uint16_t addr;
	uint16_t val;
	uint16_t op;

	for( i = 0; i < cnt; i++ ){
		addr = gen_prbs( 0, mem_size );
		val  = gen_prbs( 0, U16_MAX );
		op   = gen_prbs( 0, 2 );

		if( op == 1 ){
			*( ref_mem + addr  ) = val;
			write( context, offset + addr, val );
		} else {
			if( *( ref_mem + addr )!= read( context, offset + addr) )
				*error = *error + 1;
		}

	}
	check_mem( context, ref_mem, offset, error, mem_size );

}

void fill_buff_prbs( uint16_t * buff, int cnt, uint16_t min, uint16_t max ){
	int i;

	for( i = 0; i < cnt; i++ )
		*( buff + i ) = gen_prbs( min, max );
}

void fill_buff_cnt( uint16_t * buff, int cnt ){
	int i;

	for( i = 0; i < cnt; i++ )
		*( buff + i ) = i & U16_MAX;
}

ktime_t __attribute__((optimize("O0"))) write_cycle_bench( void * context, int cnt, uint16_t offset ){
	int i;
	ktime_t s, f;
	s = ktime_get();

	for( i = 0; i < cnt; i++ )
		write( context, offset+i, i & 0xFFFF );
	f = ktime_get();
	return div_s64( ktime_sub(f, s), cnt );
}

ktime_t __attribute__((optimize("O0"))) read_cycle_bench( void * context, int cnt, uint16_t offset ){
	int i;
	ktime_t s, f;
	s = ktime_get();


	for( i = 0; i < cnt; i++ )
		read( context, offset+i );
	f = ktime_get();
	return div_s64( ktime_sub(f, s), cnt );
}


void mem_test_speed( struct stcmtk_common_fpga * g, struct fpga_feature * mf  ){
	int i                  = 0;
	ktime_t min_time = S64_MAX;
	ktime_t max_time = 0;
	ktime_t sum_time = 0;
	ktime_t time     = 0;

	dev_notice(g->abstract_fpga, "Test speed start.");

	for( i = 0; i < REPEAT_CNT; i++ ){
		time = write_cycle_bench( g->regmap, REPEAT_CNT, mf->cr_base );
		SET_TIME_PARAM( time, max_time, min_time, sum_time );
	}
	PRINT_TIME( g->abstract_fpga, max_time, min_time, div_s64( sum_time, REPEAT_CNT ), "WRITE" );

	min_time = S64_MAX;
	max_time = 0;
	sum_time = 0;

	for( i = 0; i < REPEAT_CNT; i++ ){
		time = read_cycle_bench( g->regmap, REPEAT_CNT, mf->cr_base );
		SET_TIME_PARAM( time, max_time, min_time, sum_time );
	}
	PRINT_TIME( g->abstract_fpga, max_time, min_time, div_s64( sum_time, REPEAT_CNT ), "READ" );

}

int grif_mem_test( struct stcmtk_common_fpga * g, int iteration ) {
	int i          = 0;
	int itr        = 0;
	int error      = 0;
	int error_prev = 0;
	int ret        = 0;

	uint16_t * ref_mem = NULL;
	struct fpga_feature * mf = NULL;

	uint16_t * addr_buf = NULL;
	uint16_t * val_buf  = NULL;

	if( !stcmtk_is_fpga_feature_present( "MEM", g->features ) ){
		dev_err(g->abstract_fpga, "No feature MEM!" );
		return -ENODEV;
	}

	mf = stcmtk_get_feature_by_name( "MEM", g->features);
	if( IS_ERR(mf) ){
		dev_err(g->abstract_fpga, "Can't get struct fpga_feature for MEM!" );
		return -ENXIO;
	}

	dev_notice(g->abstract_fpga, "MEM FEAT VERSION: %d", read(g->regmap, mf->sr_base) );

	ref_mem = kzalloc( sizeof( uint16_t ) * mf->cr_cnt, GFP_KERNEL );
	if( !ref_mem ){
		dev_err(g->abstract_fpga, "Can't allocate memory for ref memory!" );
		goto free_ref_mem;
		ret = -ENOMEM;
	}

	addr_buf = kzalloc( sizeof( uint16_t ) * mf->cr_cnt, GFP_KERNEL );
	if( !addr_buf ){
		dev_err(g->abstract_fpga, "Can't allocate memory for addr memory!" );
		goto free_addr_buf;
		ret = -ENOMEM;
	}

	val_buf = kzalloc( sizeof( uint16_t ) * mf->cr_cnt, GFP_KERNEL );
	if( !val_buf ){
		dev_err(g->abstract_fpga, "Can't allocate memory for val memory!" );
		goto free_val_buf;
		ret = -ENOMEM;
	}

	for( itr = 0; itr < iteration; itr++ ){
		//Shift one tests
		shift_one( g->regmap, ref_mem, mf->cr_base, 0, &error );
		shift_one( g->regmap, ref_mem, mf->cr_base, mf->cr_cnt - 1, &error );
		for( i = 0; i < REPEAT_CNT; i++)
			shift_one( g->regmap, ref_mem, mf->cr_base, gen_prbs( 0, mf->cr_cnt - 1), &error );
		if( error != error_prev ){
			dev_err( g->abstract_fpga, "Errors after shift one test %d", error );
			error_prev = error;
		}

		dev_notice( g->abstract_fpga, "Shift one test done %i.", itr );

		//Burst read, write
		mem_dump( g->abstract_fpga, g->regmap, mf->cr_base, REPEAT_CNT );
		fill_buff_cnt( addr_buf, mf->cr_cnt );
		set_array( g->regmap, addr_buf, addr_buf, mf->cr_base, mf->cr_cnt );
		set_array_ref( ref_mem, addr_buf, addr_buf, mf->cr_cnt );
		check_mem( g->regmap, ref_mem, mf->cr_base, &error, mf->cr_cnt );
		mem_dump( g->abstract_fpga, g->regmap, mf->cr_base, REPEAT_CNT );
		if( error != error_prev ){
			dev_err(g->abstract_fpga, "Errors after wr_only, rd_only cnt test %d", error );
			error_prev = error;
		}

		dev_notice(g->abstract_fpga, "CNT wr_only, rd_only test done %i.", itr );

		//Burst read, write
		mem_dump( g->abstract_fpga, g->regmap, mf->cr_base, REPEAT_CNT );
		fill_buff_prbs( addr_buf, REPEAT_CNT, 0, mf->cr_cnt );
		fill_buff_prbs( val_buf,  REPEAT_CNT, 0, U16_MAX );
		set_array( g->regmap, addr_buf, val_buf, mf->cr_base, REPEAT_CNT );
		set_array_ref( ref_mem, addr_buf, val_buf, REPEAT_CNT );
		check_mem( g->regmap, ref_mem, mf->cr_base, &error, mf->cr_cnt );
		mem_dump( g->abstract_fpga, g->regmap, mf->cr_base, REPEAT_CNT );
		if( error != error_prev ){
			dev_err(g->abstract_fpga, "Errors after wr_only, rd_only test %d", error );
			error_prev = error;
		}

		dev_notice(g->abstract_fpga, "Random wr_only, rd_only test done %i.", itr );

		//Random read, random write
		random( g->regmap, ref_mem, mf->cr_base, &error, mf->cr_cnt, REPEAT_CNT );
		check_mem( g->regmap, ref_mem, mf->cr_base, &error, mf->cr_cnt );
		if( error != error_prev ){
			dev_err(g->abstract_fpga, "Errors random wr rd  %d", error );
			error_prev = error;
		}
		dev_notice(g->abstract_fpga, "Random wr rd done %i.", itr );
	}
	dev_notice(g->abstract_fpga, "Mem test errors: %i.", error );

	mem_test_speed( g, mf );
	dev_notice(g->abstract_fpga, "Mem test done." );

free_val_buf:
	kfree( val_buf );
free_addr_buf:
	kfree( addr_buf );
free_ref_mem:
	kfree( ref_mem );
	return ret;
}
