TARGET = madgwick_test

# Prevajalnik in zastavice (vključno z MISRA priporočenimi opozorili)
CC = gcc
CFLAGS = -Wall -Wextra -O2 -I.

# Povezovalne zastavice (matematična knjižnica je nujna!)
LDFLAGS = -lm

# Vse C datoteke v projektu
SRCS = src/main.c src/ahrs/vector_math.c src/ahrs/madgwick.c

# Samodejno generiranje imen za .o (object) datoteke
OBJS = $(SRCS:.c=.o)

# Glavno pravilo za build
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(LDFLAGS)

# Pravilo za prevajanje posameznih .c datotek v .o
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Brisanje prevedenih datotek (make clean)
clean:
	rm -f $(OBJS) $(TARGET)