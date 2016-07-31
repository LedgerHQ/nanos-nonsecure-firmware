#ifndef BOOTSECTOR_H
#define BOOTSECTOR_H

typedef void (*main_t) (void);
typedef void (*loadedmain_t) (unsigned int button_press_ms);

typedef struct bootsector_nvram_s {
#define MAIN_MAGIC 0x51145E82
  unsigned int main_magic;
  loadedmain_t main;

#define BS_MAGIC 0xFAD05E82
  unsigned int bs_magic;
  main_t bs;
} bootsector_nvram_t;

extern bootsector_nvram_t N_bootsector __attribute__ ((section(".bootsector_nvram")));

#ifndef ARM_CORE_HAS_VTOR
typedef struct bootsector_ram_s {
  // array of ISR handlers
  main_t *VTOR;
} bootsector_ram_t;

extern bootsector_ram_t G_bootsector __attribute__ ((section(".bootsector_ram")));
#endif // !ARM_CORE_HAS_VTOR

#endif // BOOTSECTOR_H
