#include "mednafen/tremor/tremor_shared.c"
#include "mednafen/tremor/codebook.c"
#include "mednafen/tremor/floor0.c"
#include "mednafen/tremor/floor1.c"
#include "mednafen/tremor/mdct.c"
#include "mednafen/tremor/registry.c"
#include "mednafen/tremor/mapping0.c"
#include "mednafen/tremor/info.c"
#include "mednafen/tremor/res012.c"
#include "mednafen/tremor/framing.c"
#include "mednafen/tremor/block.c"
#include "mednafen/tremor/sharedbook.c"
#include "mednafen/tremor/synthesis.c"
#include "mednafen/tremor/vorbisfile.c"
#include "mednafen/tremor/bitwise.c"
#include "mednafen/tremor/window.c"

#include "mednafen/trio/trio.c"
#include "mednafen/trio/triostr.c"

#include "threads.c"
#include "scrc32.c"

#if NEED_CD
#include "mednafen/cdrom/galois.c"
#endif
