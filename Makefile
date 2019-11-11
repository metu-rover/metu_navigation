#!/usr/bin/make
#
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
# METU 19th Dormitory, Suite 329-2, Ankara, 06800, Turkey.					#
#                                                                           #
# ------------------------------------------------------------------------- #

ifndef PACK
PACK:=default
endif

workspace:=rover_ws
TEMPLATE:=$(HOME)/Templates/
PACKPATH:=$(HOME)/$(workspace)/$(PACK)/

%.py:
	@cp -nv $(TEMPLATE)python.py $(PACKPATH)lib/$@

__init__.py:
	@cp -nv $(TEMPLATE)python.py $(PACKPATH)src/$@

main_%.c:
	@cp -nv $(TEMPLATE)c\ source\ file.c $(PACKPATH)src/$@
	@sed -i 's/mylibrary/..\/lib\/$*/; /template<class T>/,+6 d' $(PACKPATH)src/$@

__main__.c:
	@cp -nv $(TEMPLATE)c\ source\ file.c $(PACKPATH)src/$@
	@sed -i 's/mylibrary/..\/lib\/$(PACK)/; /MyFunction/,+5 d' $(PACKPATH)src/$@

%.c:
	@cp -nv $(TEMPLATE)c\ source\ file.c $(PACKPATH)lib/$@
	@sed -i 's/mylibrary/$*/; s/MyFunction/$*/; /main/,+7 d' $(PACKPATH)lib/$@

%.h: %.c
	@cp -nv $(TEMPLATE)c\ header\ file.h $(PACKPATH)lib/$@
	@sed -i 's/HEADER_H/$*_h/g; s/MyTemplateStruct/$*/' $(PACKPATH)lib/$@

main_%.cpp:
	@cp -nv $(TEMPLATE)c++\ source\ file.cpp $(PACKPATH)src/$@
	@sed -i 's/mylibrary/..\/lib\/$*/; /template<class T>/,+6 d' $(PACKPATH)src/$@

__main__.cpp:
	@cp -nv $(TEMPLATE)c++\ source\ file.cpp $(PACKPATH)src/$@
	@sed -i 's/mylibrary/..\/lib\/$(PACK)/; /template<class T>/,+6 d' $(PACKPATH)src/$@

%.cpp:
	@cp -nv $(TEMPLATE)c++\ source\ file.cpp $(PACKPATH)lib/$@
	@sed -i 's/mylibrary/$*/ ; s/MyTemplateFunction/$*/g; /main/,+7 d' $(PACKPATH)lib/$@

%.hpp: %.cpp
	@cp -nv $(TEMPLATE)c++\ header\ file.hpp $(PACKPATH)lib/$@
	@sed -i 's/HEADER_HPP/$*_hpp/g; s/MyTemplateClass/$*/g' $(PACKPATH)lib/$@

%.sh:
	@cp -nv $(TEMPLATE)sh\ file.sh $(PACKPATH)src/$@
	@sed -i 's/untitled\ sh\ file.sh/$@/; s/anywhere/$(PACK)/' $(PACKPATH)src/$@

%.inp:
	@touch $(PACKPATH)buf/inp/$@

%.out:
	@touch $(PACKPATH)buf/out/$@

%.err:
	@touch $(PACKPATH)buf/err/$@

%.txt:
	@touch $(PACKPATH)usr/$@

.PHONY: License license build version upgrade
build:
	@mkdir -pv $(PACKPATH)bin $(PACKPATH)src $(PACKPATH)lib $(PACKPATH)buf/inp \
			   $(PACKPATH)buf/out $(PACKPATH)buf/err $(PACKPATH)usr $(PACKPATH)obj
	@cp -nv $(TEMPLATE)makefile $(PACKPATH)
	@sed -i 's/_ws/$(workspace)/g; s/SOMEPCK/$(PACK)/g' $(PACKPATH)makefile
	@chmod -Rv og-wrx $(PACKPATH)*
	@echo	"You are supposed to have a main file:\n" \
			"\n" \
			"\t\t* make __init__.py PACK=$(PACK)\n" \
			"\t\t* make __main__.cpp PACK=$(PACK)\n" \
			"\t\t* make __main__.c  PACK=$(PACK)\n"

version:
	@echo beta-1.01.1a

update:
	@echo it is-up-to-date

upgrade:
	@echo everything is up-to-date

License license:
	@echo "\n" \
	"License:\n\n" \
	\
	"\tThis program is free software; you can redistribute it and/or modify\n" \
	"it under the terms of the GNU General Public License as published by\n" \
	"the Free Software Foundation; either version 3, or (at your option)\n" \
	"any later version.\n\n" \
	\
	"\tThis program is distributed in the hope that it will be useful, but\n" \
	"WITHOUT ANY WARRANTY; without even the implied warranty of\n" \
	"MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU\n" \
	"General Public License for more details.\n\n" \
	\
	"\tYou should have received a copy of the GNU General Public License\n" \
	"along with this program; if not, write to the Free Software Foundation, \n" \
	"METU 19th Dormitory, Suite 329-2, Ankara, 06800, Turkey.\n"

