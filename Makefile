.SUFFIXES: .cpp .hpp

CPPFLAGS = -g -O3 -Wall -fno-strict-aliasing -I ./external/include -I /usr/include
LDFLAGS = -L ./external/lib -L /usr/lib

LIBS = -lBox2D -lglui -lglut -lGLU -lGL

SOURCES := $(wildcard src/*.cpp)
OBJECTS := $(SOURCES:src/%.cpp=myobjs/%.o)

LIB_OBJECTS = $(OBJECTS)
LIB_OBJECTS-= myobjs/main.o
INSTALL_DIR = ../../final

SHARED_LIB = TRUE

.PHONY: all setup exe static_lib dynamic_lib exelib clean distclean doc report

all: exe exelib

setup:
	@mkdir -p myobjs mylibs mybins
	@if [ ! -d external/include/Box2D ]; \
	then \
	tar xzf external/src/Box2D.tgz --directory external/src; \
	mkdir external/src/Box2D/build296; \
	cd external/src/Box2D/build296; \
	cmake ../; \
	make; \
	make install; \
	fi;

$(OBJECTS) : myobjs/%.o : src/%.cpp
	@g++ $(CPPFLAGS) -fPIC -c $< -o $@ -MD
	
exe: setup $(OBJECTS)
	@g++ -o mybins/cs296_22_exe $(LDFLAGS) myobjs/*.o $(LIBS)

static_lib: $(OBJECTS)
ifeq ($(SHARED_LIB),FALSE)
	@ar rs mylibs/libCS296test.a $(LIB_OBJECTS)
endif

dynamic_lib: $(OBJECTS)
ifeq ($(SHARED_LIB),TRUE)
	@g++ -shared -o mylibs/libCS296test.so $(LIB_OBJECTS)
endif

exelib: setup $(OBJECTS)
ifeq ($(SHARED_LIB),FALSE)
	@make static_lib
	@g++ -o mybins/cs296_22_exelib $(LDFLAGS) myobjs/main.o mylibs/libCS296test.a $(LIBS)
else
	@make dynamic_lib
	@g++ -o mybins/cs296_22_exelib myobjs/main.o $(LDFLAGS) mylibs/libCS296test.so $(LIBS)
endif

doc:
	@echo -n "Generating Doxygen Documentation ...  "
	@rm -rf doc/html
	@doxygen doc/Doxyfile 2 > /dev/null
	@echo "Done"

report:
	@cd doc; \
	pdflatex g22_project_report; \
	bibtex g22_project_report; \
	pdflatex g22_project_report; \
	pdflatex g22_project_report; \
	rm g22_project_report.aux g22_project_report.log g22_project_report.blg g22_project_report.bbl;
	@python3 scripts/g22_gen_html.py; \

dist: distclean
	@cd ../../; \
	tar cvzf cs296_g22_project.tar.gz g22_project README.txt

install: exe doc report
	@rm -rf $(INSTALL_DIR)
	@mkdir $(INSTALL_DIR)
	@cp -r mybins/ $(INSTALL_DIR)
	@cp -r mylibs/ $(INSTALL_DIR)
	@cp -r doc/ $(INSTALL_DIR)
	@cp -r plots/ $(INSTALL_DIR)
	@cp -r scripts/ $(INSTALL_DIR)

clean:
	@rm -rf myobjs mybins mylibs

distclean: clean
	@rm -rf external/include/* external/lib/* external/src/Box2D doc/html doc/g22_project_report.pdf doc/g22_project_report.html
