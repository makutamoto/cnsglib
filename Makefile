SRCS := $(wildcard *.c)
OBJS_CLANG := $(patsubst %.c, objs/%.o, $(SRCS))
OBJS_BCC32 := $(patsubst %.c, objs/%.obj, $(SRCS))

clang: $(OBJS_CLANG)
	@echo "Build complete."

bcc32: $(OBJS_BCC32)
	@echo "Build complete."

clean:
	del objs\*.o objs\*.obj

objs/%.o: %.c ./include/*.h
	clang -Wall -c $< -o $@ -Wno-invalid-source-encoding

objs/%.obj: %.c ./include/*.h
	bcc32 -wAll -o"$@" -c $<
