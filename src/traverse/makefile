#!/usr/bin/make
# ------------------------ # MAKEFILE IN traverse # ------------------------ #
# ------------------------------ # LICENSE # ------------------------------ #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2, or (at your option)       #
# any later version.                                                        #
#                                                                           #
# This program is distributed in the hope that it will be useful, but       #
# WITHOUT ANY WARRANTY; without even the implied warranty of                #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU          #
# General Public License for more details.                                  #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software Foundation,   #
# 51 Franklin Street, Fifth Floor Boston, MA 02110 USA.                     #
#                                                                           #
# ------------------------------------------------------------------------- #
# ------------------------ # STANDARD  VARIABLES # ------------------------ #

version:=ahd-1.3.01
_WrS   :=rover_ws
_GCC   :=gcc 
_GXX   :=g++ 
_CFLAGS:=-c -g -ansi -Wall -pedantic-errors -O0

ifneq (${val},no)
_ValG  :=valgrind --leak-check=full --show-leak-kinds=all -v ${val} 
endif

ifneq (${comp},no)
_GCC   +=-v ${comp} 
_GXX   +=-v -std=c++11 ${comp} 
endif

ifdef inp
_StdI  := < ${HOME}/${_WrS}/traverse/buf/inp/${inp} 
endif

ifdef out
_StdO  := > ${HOME}/${_WrS}/traverse/buf/out/${out} 
endif

ifdef err
_StdE  := 2> ${HOME}/${_WrS}/traverse/buf/err/${err} 
_GXX   += ${_StdE}
_GCC   += ${_StdE}
endif

ifdef arg
_ArgI  := ${arg} 
endif

# ----------------------- # INTERACTIVE VARIABLES # ----------------------- #

all=
all_deps=

all: ${all}

# ------------------------ # USER-DEFINED  RULES # ------------------------ #

# -------------------------- # STANDARD  RULES # -------------------------- #
# -------------------------- # EXECUTION RULES # -------------------------- #

%: bin/%
	${_ValG} $^ ${_StdI}${_StdO}${_StdE}${_ArgI}

# --------------------------- # BINARY  RULES # --------------------------- #

bin/Main: src/__main__.c
	${_GCC} $^ -o $@

bin/main: src/__main__.cpp
	${_GXX} $^ -o $@

bin/%: obj/main_%.o
	${_GXX} $^ -o $@

bin/t_%: obj/test_%.o
	${_GXX} $^ -o $@

# --------------------------- # OBJECT  RULES # --------------------------- #

obj/main_%.o: src/main_%.cpp
	${_GXX} ${_CFLAGS} $^ -o $@

obj/main_%.o: src/main_%.c
	${_GCC} ${_CFLAGS} $^ -o $@

obj/test_%.o: src/test_%.cpp
	${_GXX} ${_CFLAGS} $^ -o $@

obj/test_%.o: src/test_%.c
	${_GCC} ${_CFLAGS} $^ -o $@

obj/%.o: inc/%.cpp
	${_GXX} ${_CFLAGS} $^ -o $@

obj/%.o: inc/%.c
	${_GCC} ${_CFLAGS} $^ -o $@

obj/%.o: inc/%.cpp lib/%.hpp
	${_GXX} ${_CFLAGS} $< -o $@

obj/%.o: inc/%.c lib/%.h
	${_GCC} ${_CFLAGS} $< -o $@

# -------------------------- # CLEANING  RULES # -------------------------- #

clean:
	@rm -iv obj/* bin/*

clean-%:
	@rm -iv bin/$*

clean-%.o:
	@rm -iv obj/$*.o

# --------------------------- # REST OF RULES # --------------------------- #

.PHONY: build clean license License version Version all clean-% clean-%.o
Version version:
	@echo ${version}

License license:
	@echo "\n" \
	"License: written by Adnan Harun DOĞAN\n" \
	"\n" \
	"This program is free software; you can redistribute it and/or modify\n" \
	"it under the terms of the GNU General Public License as published by\n" \
	"the Free Software Foundation; either version 3, or (at your option)\n" \
	"any later version.\n" \
	"\n" \
	"This program is distributed in the hope that it will be useful, but\n" \
	"WITHOUT ANY WARRANTY; without even the implied warranty of\n" \
	"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU\n" \
	"General Public License for more details.\n" \
	"\n" \
	"You should have received a copy of the GNU General Public License\n" \
	"along with this program; if not, write to the Free Software Foundation, \n" \
	"51 Franklin Street, Fifth Floor Boston, MA 02110 USA.\n"

# -------------------------- # END OF MAKEFILE # -------------------------- #
