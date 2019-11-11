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
# METU 19th Dormitory, Suite 329-2, Ankara, 06800, Turkey.                  #
#                                                                           #
# ------------------------------------------------------------------------- #
# ------------------------ # CONSTANT  VARIABLES # ------------------------ #

ifndef PACK
  PACK :=traverse
endif

version     :=ahd-1.0.01
workspace   :=rover_ws
C           :=gcc
CC          :=g++
CFLAGS      :=-c -ansi -Wall -pedantic-errors -O0
PACKPATH    := $(HOME)/$(workspace)/$(PACK)/
TEMPLATE    :=$(HOME)/Templates/
TRASHBIN    :=$(HOME)/src_master/.trash/

# ----------------------- # INTERACTIVE VARIABLES # ----------------------- #

all=
all_deps=

all: $(all)
	@valgrind --leak-check=full $^

# ----------------------------- # SRC-RULES # ----------------------------- #
# --------------------------- # REST OF RULES # --------------------------- #

%: bin/%
	@valgrind --leak-check=full $^

bin/%: src/main_%.o
	$(CC) $^ -o $@

obj/%.o: lib/%.cpp lib/%.hpp
	$(CC) $(CFLAGS) $< -o $@

obj/%.o: lib/%.cpp
	$(CC) $(CFLAGS) $^ -o $@

.PHONY: build clean license License version all
clean:
	@rm -iv $(all) $(all_deps)

version:
	@echo $(version)

License license:
	@echo "\n" \
	"License:\n\n" \
	\
	"\tThis program is free software; you can redistribute it and/or modify\n" \
	"it under the terms of the GNU General Public License as published by\n" \
	"the Free Software Foundation; either version 3, or (at your option)\n" \
	"any later version.\n" \
	"\n" \
	"\tThis program is distributed in the hope that it will be useful, but\n" \
	"WITHOUT ANY WARRANTY; without even the implied warranty of\n" \
	"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU\n" \
	"General Public License for more details.\n" \
	"\n" \
	"\tYou should have received a copy of the GNU General Public License\n" \
	"along with this program; if not, write to the Free Software Foundation, \n" \
	"METU 19th Dormitory, Suite 329-2, Ankara, 06800, Turkey.\n"

# -------------------------- # END OF MAKEFILE # -------------------------- #
