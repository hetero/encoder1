#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <x86intrin.h>

#include "mjpeg_encoder.h"

static char *output_file;
static char *input_file;
static FILE *outfile;

static int limit_numframes = 0;

static uint32_t width;
static uint32_t height;
static uint32_t yph;
static uint32_t ypw;
static uint32_t uph;
static uint32_t upw;
static uint32_t vph;
static uint32_t vpw;

static uint32_t bit_buffer = 0;
static uint32_t bit_buffer_width = 0;

/* getopt */
extern int optind;
extern char *optarg;

/* cosines memorizing */
static float cos_table[4096 * 3] __attribute__((aligned(16)));

static dct_t *out;
static uint8_t *byte_array;

static void calc_cos_table() {
	int u, v, j, i;
	for (i = 0; i < 8; i++)
		for (u = 0; u < 8; u++)
			for (v = 0; v < 8; v++)
				for (j = 0; j < 8; j++) {
					float q1 = Fyquanttbl[v * 8 + u];
					float q2 = Fuquanttbl[v * 8 + u];
					float q3 = Fvquanttbl[v * 8 + u];
					float tmp_cos = cos((2 * i + 1) * v * PI / 16.0f)
							* cos((2 * j + 1) * u * PI / 16.0f);
					float a1 = !u ? ISQRT2 : 1.0f;
					float a2 = !v ? ISQRT2 : 1.0f;
					cos_table[512*i + 64*v + 8*u + j] = tmp_cos * a1 * a2 / (q1 * 4.0f);
					cos_table[4096 + 512*i + 64*v + 8*u + j] = tmp_cos * a1 * a2 / (q2 * 4.0f);
					cos_table[2 * 4096 + 512*i + 64*v + 8*u + j] = tmp_cos * a1 * a2 / (q3 * 4.0f);
				}
}

/* Read YUV frames */
void read_yuv(FILE *file, yuv_t *image) {
	size_t len = 0;
	int i;

	/* Read Y' */
	len += fread(byte_array, 1, width * height, file);
	if (ferror(file)) {
		perror("ferror");
		exit(EXIT_FAILURE);
	}
	for (i = 0; i < width*height; ++i)
		image->Yf[i] = (byte_array[i] - 128);

	/* Read U */
	len += fread(byte_array, 1, width * height / 4, file);
	if (ferror(file)) {
		perror("ferror");
		exit(EXIT_FAILURE);
	}
	for (i = 0; i < width*height/4; ++i)
		image->Uf[i] = (byte_array[i] - 128);

	/* Read V */
	len += fread(byte_array, 1, width * height / 4, file);
	if (ferror(file)) {
		perror("ferror");
		exit(EXIT_FAILURE);
	}
	for (i = 0; i < width*height/4; ++i)
		image->Vf[i] = (byte_array[i] - 128);

	if (len != width * height * 1.5) {
		printf("Reached end of file.\n");
	}

}

static void dct_quantize(float *in_data, uint32_t width, uint32_t height,
        float *out_data, uint32_t padwidth,
        uint32_t padheight, uint8_t id_quant)
{
	int y, x, u, v, i;
	//const __m128 f128_xmm0 = _mm_set1_ps(128);
	__m128 in_xmm1, cos_xmm2, in_xmm3, cos_xmm4;
    __m128 reg[16];
	float *out_ptr;
	float tmp[64] __attribute__((aligned(16)));
	//float table[4] __attribute__((aligned(16)));

	/* Perform the DCT and quantization */
	for (y = 0; y < height; y += 8) {
		int jj = height - y;
		jj = MIN(jj, 8); // For the border-pixels, we might have a part of an 8x8 block

		for (x = 0; x < width; x += 8) {
			int ii = width - x;
			ii = MIN(ii, 8); // For the border-pixels, we might have a part of an 8x8 block

			float *in_ptr = &in_data[y * width + x];
			out_ptr = &out_data[y * width + x];
			if (ii == 8 && jj == 8) {
				for (i = 0; i < 64; ++i) {
					tmp[i] = 0.0f;
				}
				for (i = 0; i < 16; ++i)
					reg[i] = _mm_set1_ps(0.5);
				//Loop through all elements of the in block, vector by vector
				for (i = 0; i < 8; ++i) {
					in_xmm1 = _mm_load_ps(in_ptr);
					in_xmm3 = _mm_load_ps(in_ptr + 4);
					float *cos_ptr = &cos_table[id_quant * 4096 + 512 * i];
					//Loop through all elements of the out block
					for (v = 0; v < 8; ++v) {
						for (u = 0; u < 8; u++) {
							cos_xmm2 = _mm_load_ps(cos_ptr + v * 64 + u * 8);
							cos_xmm2 = _mm_dp_ps(in_xmm1, cos_xmm2, 240 + (1 << (u % 4)));
							cos_xmm4 = _mm_load_ps(cos_ptr + v * 64 + u * 8 + 4);
							cos_xmm4 = _mm_dp_ps(in_xmm3, cos_xmm4, 240 + (1 << (u % 4)));
							cos_xmm4 = _mm_add_ps(cos_xmm4, cos_xmm2);
							reg[2 * v + u / 4] = _mm_add_ps(reg[2 * v + u / 4], cos_xmm4);
						}
					}
					in_ptr += width;
				}
				for (v = 0; v < 8; ++v) {
					for (u = 0; u < 2; u++) {
						_mm_storeu_ps(out_ptr, reg[2 * v + u]);
						out_ptr+= 4;
					}
					out_ptr += width - 8;
				}
			} else { // border case
				int j;
				float dct, coeff;
				for(v = 0; v << ii; ++v)
				{
					for (u = 0; u < jj; ++u)
					{
						dct = 0;
						for (i = 0; i < ii; ++i)
						{
							for (j = 0; j < jj; ++j)
							{
								coeff = in_data[(y + i) * width + (x + j)];
								dct += coeff * cos_table[id_quant * 4096 + 512 * i + 64 * v + 8 * u + j];
							}
						}
						*out_ptr = (float) (0.5f + dct);
						out_ptr++;
					}
					out_ptr += width - jj;
				}
			}
		}
	}
}

static void put_byte(int byte) {
	int status = fputc(byte, outfile);

	if (status == EOF) {
		fprintf(stderr, "Error writing byte\n");
		exit(EXIT_FAILURE);
	}
}

static void put_bytes(const void* data, unsigned int len) {
	int n = fwrite(data, 1, len, outfile);

	if (n != len) {
		fprintf(stderr, "Error writing bytes\n");
		exit(-1);
	}
}

static void write_SOI() {
	put_byte(0xff);
	put_byte(0xd8);
}

static void write_DQT() {
	int16_t size = 2 + (3 * 65);

	put_byte(0xff);
	put_byte(0xdb);

	put_byte(size >> 8);
	put_byte(size & 0xff);

	put_byte(0);
	put_bytes(yquanttbl, 64);
	put_byte(1);
	put_bytes(uquanttbl, 64);
	put_byte(2);
	put_bytes(vquanttbl, 64);
}

static void write_SOF0() {
	int16_t size = 8 + 3 * COLOR_COMPONENTS;

	/* Header marker */
	put_byte(0xff);
	put_byte(0xc0);

	/* Size of header */
	put_byte(size >> 8);
	put_byte(size & 0xff);

	/* Precision */
	put_byte(8);

	/* Width and height */
	put_byte(height >> 8);
	put_byte(height & 0xff);
	put_byte(width >> 8);
	put_byte(width & 0xff);

	put_byte(COLOR_COMPONENTS);

	put_byte(1); /* Component id */
	put_byte(0x22); /* hor | ver sampling factor FIXME Y(2,2), U(1,1), V(1,1) */
	put_byte(0); /* Quant. tbl. id */

	put_byte(2); /* Component id */
	put_byte(0x11); /* hor | ver sampling factor */
	put_byte(1); /* Quant. tbl. id */

	put_byte(3); /* Component id */
	put_byte(0x11); /* hor | ver sampling factor */
	put_byte(2); /* Quant. tbl. id */
}

static void write_DHT_HTS(uint8_t id, uint8_t *numlength, uint8_t* data) {
	/* Find out how many codes we are to write */
	int i;
	int n = 0;
	for (i = 0; i < 16; ++i)
		n += numlength[i];

	put_byte(id);
	put_bytes(numlength, 16);
	put_bytes(data, n);
}

static void write_DHT() {
	int16_t size = 0x01A2; /* 2 + n*(17+mi); */

	/* Define Huffman Table marker */
	put_byte(0xff);
	put_byte(0xc4);

	/* Length of segment */
	put_byte(size >> 8);
	put_byte(size & 0xff);

	/* Write the four huffman table specifications */
	write_DHT_HTS(0x00, DCVLC_num_by_length[0], DCVLC_data[0]); /* DC table 0 */
	write_DHT_HTS(0x01, DCVLC_num_by_length[1], DCVLC_data[1]); /* DC table 1 */
	write_DHT_HTS(0x10, ACVLC_num_by_length[0], ACVLC_data[0]); /* AC table 0 */
	write_DHT_HTS(0x11, ACVLC_num_by_length[1], ACVLC_data[1]); /* AC table 1 */
}

static void write_SOS() {
	int16_t size = 6 + 2 * COLOR_COMPONENTS;

	put_byte(0xff);
	put_byte(0xda);

	put_byte(size >> 8);
	put_byte(size & 0xff);

	put_byte(COLOR_COMPONENTS);

	put_byte(1); /* Component id */
	put_byte(0x00); /* DC | AC huff tbl */
	put_byte(2); /* Component id */
	put_byte(0x11); /* DC | AC huff tbl */
	put_byte(3); /* Component id */
	put_byte(0x11); /* DC | AC huff tbl */
	put_byte(0); /* ss, first AC */
	put_byte(63); /* se, last AC */
	put_byte(0); /* ah | al */
}

static void write_EOI() {
	put_byte(0xff);
	put_byte(0xd9);
}

static inline uint8_t bit_width(int16_t i) {
	return (uint8_t) (ceil((log(abs(i) + 1)) * ILOG2));
}

/**
 * Adds a bit to the bitBuffer. A call to Flush() is needed
 * in order to write any remainding bits in the buffer before
 * writing using another function.
 */
static void put_bits(int16_t bits, uint8_t n) {
	assert(n <= 24 && "Error writing bit");

	if (n == 0)
		return;

	bit_buffer <<= n;
	bit_buffer |= bits & ((1 << n) - 1);
	bit_buffer_width += n;

	while (bit_buffer_width >= 8) {
		uint8_t c = (uint8_t) (bit_buffer >> (bit_buffer_width - 8));
		put_byte(c);
		if (c == 0xff)
			put_byte(0);

		bit_buffer_width -= 8;
	}
}

/**
 * Flushes the bitBuffer by writing zeroes to fill a full byte
 */
static void flush() {
	if (bit_buffer > 0) {
		uint8_t c = bit_buffer << (8 - bit_buffer_width);
		put_byte(c);
		if (c == 0xff)
			put_byte(0);
	}

	bit_buffer = 0;
	bit_buffer_width = 0;
}

static void write_block(float *in_data, uint32_t width, uint32_t height,
		uint32_t uoffset, uint32_t voffset, int16_t *prev_DC, int32_t cc) {
	uint32_t zigzag, i, j;

	static int16_t block[64];
	int32_t num_ac = 0;

	/* ZigZag */
	for (zigzag = 0; zigzag < 64; zigzag++) {
		uint8_t u = zigzag_U[zigzag];
		uint8_t v = zigzag_V[zigzag];
		block[zigzag] = in_data[(voffset + v) * width + (uoffset + u)];
	}

	/* Calculate DC component, and write to stream */
	int16_t dc = block[0] - *prev_DC;
	*prev_DC = block[0];
	uint8_t size = bit_width(dc);
	put_bits(DCVLC[cc][size], DCVLC_Size[cc][size]);
	if (dc < 0)
		dc = dc - 1;

	put_bits(dc, size);

	/* find the last nonzero entry of the ac-coefficients */
	for (j = 64; j > 1 && !block[j - 1]; j--)
		;

	/* Put the nonzero ac-coefficients */
	for (i = 1; i < j; i++) {
		int16_t ac = block[i];
		if (ac == 0) {
			if (++num_ac == 16) {
				put_bits(ACVLC[cc][15][0], ACVLC_Size[cc][15][0]);
				num_ac = 0;
			}
		} else {
			uint8_t size = bit_width(ac);
			put_bits(ACVLC[cc][num_ac][size], ACVLC_Size[cc][num_ac][size]);

			if (ac < 0)
				--ac;
			put_bits(ac, size);
			num_ac = 0;
		}
	}

	/* Put end of block marker */
	if (j < 64)
		put_bits(ACVLC[cc][0][0], ACVLC_Size[cc][0][0]);
}

static void write_interleaved_data_MCU(float *dct, uint32_t wi, uint32_t he,
		uint32_t h, uint32_t v, uint32_t x, uint32_t y, int16_t *prev_DC,
		int32_t cc) {
	uint32_t i, j, ii, jj;
	for (j = y * v * 8; j < (y + 1) * v * 8; j += 8) {
		jj = he - 8;
		jj = MIN(j, jj);

		for (i = x * h * 8; i < (x + 1) * h * 8; i += 8) {
			ii = wi - 8;
			ii = MIN(i, ii);

			write_block(dct, wi, he, ii, jj, prev_DC, cc);
		}
	}
}

static void write_interleaved_data(dct_t *out) {
	int16_t prev_DC[3] = { 0, 0, 0 };
	uint32_t u, v;

	/* Set up which huffman tables we want to use */
	int32_t yhtbl = 0;
	int32_t uhtbl = 1;
	int32_t vhtbl = 1;

	/* Find the number of MCU's for the intensity */
	uint32_t ublocks = (uint32_t) (ceil(ypw / (float) (8.0f * YX)));
	uint32_t vblocks = (uint32_t) (ceil(yph / (float) (8.0f * YY)));

	/* Write the MCU's interleaved */
	for (v = 0; v < vblocks; ++v) {
		for (u = 0; u < ublocks; ++u) {
			write_interleaved_data_MCU(out->Ydctf, ypw, yph, YX, YY, u, v,
					&prev_DC[0], yhtbl);
			write_interleaved_data_MCU(out->Udctf, upw, uph, UX, UY, u, v,
					&prev_DC[1], uhtbl);
			write_interleaved_data_MCU(out->Vdctf, vpw, vph, VX, VY, u, v,
					&prev_DC[2], vhtbl);
		}
	}

	flush();
}

static void encode(yuv_t *image) {
	/* DCT and Quantization */
    dct_quantize(image->Yf, width, height, out->Ydctf, ypw, yph, 0);
    dct_quantize(image->Uf, (width*UX/YX), (height*UY/YY), out->Udctf, upw, uph, 1);
    dct_quantize(image->Vf, (width*VX/YX), (height*VY/YY), out->Vdctf, vpw, vph, 2);

	/* Write headers */

	/* Start Of Image */
	write_SOI();
	/* Define Quantization Table(s) */
	write_DQT();
	/* Start Of Frame 0(Baseline DCT) */
	write_SOF0();
	/* Define Huffman Tables(s) */
	write_DHT();
	/* Start of Scan */
	write_SOS();

	write_interleaved_data(out);

	/* End Of Image */
	write_EOI();

}

static void print_help() {
	fprintf(stderr, "Usage: ./mjpeg_encoder [options] input_file\n");
	fprintf(stderr, "Commandline options:\n");
	fprintf(stderr,
			"  -h                             height of images to compress\n");
	fprintf(stderr,
			"  -w                             width of images to compress\n");
	fprintf(stderr, "  -o                             Output file (.mjpg)\n");
	fprintf(
			stderr,
			"  [-f]                           Limit number of frames to encode\n");
	fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

int main(int argc, char **argv) {
	int nothing;
	calc_cos_table();

	int c;
	yuv_t *image;

	if (argc == 1) {
		print_help();
		exit(EXIT_FAILURE);
	}

	while ((c = getopt(argc, argv, "h:w:o:f:i:")) != -1) {
		switch (c) {
		case 'h':
			height = atoi(optarg);
			break;
		case 'w':
			width = atoi(optarg);
			break;
		case 'o':
			output_file = optarg;
			break;
		case 'f':
			limit_numframes = atoi(optarg);
			break;
		default:
			print_help();
			break;
		}
	}

	if (optind >= argc) {
		fprintf(stderr, "Error getting program options, try --help.\n");
		exit(EXIT_FAILURE);
	}

	outfile = fopen(output_file, "wb");
	if (outfile == NULL)
	{
		perror("fopen");
		exit(EXIT_FAILURE);
	}

	/* Calculate the padded width and height */
	ypw = (uint32_t) (ceil(width / 8.0f) * 8);
	yph = (uint32_t) (ceil(height / 8.0f) * 8);
	upw = (uint32_t) (ceil(width * UX / (YX * 8.0f)) * 8);
	uph = (uint32_t) (ceil(height * UY / (YY * 8.0f)) * 8);
	vpw = (uint32_t) (ceil(width * VX / (YX * 8.0f)) * 8);
	vph = (uint32_t) (ceil(height * VY / (YY * 8.0f)) * 8);

	input_file = argv[optind];

	if (limit_numframes)
		printf("Limited to %d frames.\n", limit_numframes);

	FILE *infile = fopen(input_file, "rb");

	if (infile == NULL)
	{
		perror("fopen");
		exit(EXIT_FAILURE);
	}

	/* Encode input frames */
	int numframes = 0;
	
	byte_array = malloc(width*height);
	
	image = malloc(sizeof(yuv_t));
	if (image) {
		out = malloc(sizeof(dct_t));
		
		nothing = posix_memalign((void **) &(image->Yf), 16, width * height * sizeof(float));
		nothing = posix_memalign((void **) &(image->Uf), 16, width * height * sizeof(float) / 4);
		nothing = posix_memalign((void **) &(image->Vf), 16, width * height * sizeof(float) / 4);
		
		nothing = posix_memalign((void **) &(out->Ydctf), 16, yph * ypw * sizeof(float));
		nothing = posix_memalign((void **) &(out->Udctf), 16, uph * upw * sizeof(float));
		nothing = posix_memalign((void **) &(out->Vdctf), 16, vph * vpw * sizeof(float));
		
		
		while (!feof(infile)) {
			read_yuv(infile, image);

			printf("Encoding frame %d, ", numframes);
			encode(image);

			printf("Done!\n");

			++numframes;
			if (limit_numframes && numframes >= limit_numframes)
				break;
		}
		free(image->Yf);
		free(image->Uf);
		free(image->Vf);
		free(image);
		
		free(out->Ydctf);
		free(out->Udctf);
		free(out->Vdctf);
		free(out);
	}

	fclose(outfile);
	fclose(infile);

	return EXIT_SUCCESS;
}
