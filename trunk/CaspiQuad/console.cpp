#if SUPPORT_CONSOLE
const char roll_name[]       PROGMEM = "roll";
const char pitch_name[]      PROGMEM = "pitch";
const char yaw_name[]        PROGMEM = "yaw";
const char roll_pitch_name[] PROGMEM = "roll/pitch";

PGM_P rotation_names[NUM_ROTATIONS] PROGMEM = 
{
  roll_name,
  pitch_name,
  yaw_name
};

void PrintPgm::print_pgm(const prog_char *p_pgm_str)

{
  char c;

  
  while ((c = pgm_read_byte(p_pgm_str)) != '\0')
  {
    if (c == '\n')
      print('\r');
    print(c);
    ((char *)p_pgm_str)++;
  }
}
#endif
