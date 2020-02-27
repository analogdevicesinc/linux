CC=gcc

LNXWRP_RESS_UT=lnxwrp_resources_ut
OBJ=lnxwrp_resources

INC_PATH=
LIB_PATH=

INC=$(addprefix -I,$(INC_PATH))
LIB=$(addprefix -L,$(LIB_PATH))

CFLAGS= -gdwarf-2 -g -O0 -Wall
XFLAGS= -DFMAN_RESOURCES_UNIT_TEST

all: $(LNXWRP_RESS_UT)

$(LNXWRP_RESS_UT):$(addsuffix .o,$(OBJ)) $(LNXWRP_RESS_UT).o
	$(CC) -o $(LNXWRP_RESS_UT) $(LNXWRP_RESS_UT).o $(addsuffix .o,$(OBJ))

%.o: %.c
	@(echo "        (CC)  $@")
	@($(CC) $(INC) $(CFLAGS) $(XFLAGS) -o $(@) -c $<)

.PHONY: clean

clean:
	rm -f *.o
	rm -f $(LNXWRP_RESS_UT)
