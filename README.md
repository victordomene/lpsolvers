# LP Solving Benchmarks

This repository contains a few different implementations of LP solvers,
including Simplex with several pivot rules, Interior Point Methods, and the
Ellipsoid Algorithm. Benchmarks will be performed in the future.

## Building Instructions

The following instructions are for Mac OS X. There is no guarantee that this will
work on Linux environments (but there is no particular reasons why it shouldn't).

To build, you will need CMake. This can be installed easily via `brew` or `apt-get`.

Once you have that, the following commands should work:

``
    mkdir Debug
    cd Debug
    cmake ..
    make
``

This will generate the `lpsolver` file on the `Debug` directory. The reason why
we force a Debug compilation is simply because FLENS has code that is a bit
older, and compilers will get annoyed in the Release option.

* If you want to run without printing the tableaus and debugging information,
simply edit the `CMakeLists.txt` file and remove the definition `-DDEBUG`.

Once that works, you can run the lpsolver as follows. In the `Debug` subdirectory,

``
    ./lpsolver [opt: bland/dantzig/random] <filename>
``

which will output the solution to the linear program specified by `filename`,
an MPS file. (If the file is invalid, the code will assert; this has yet to be
fixed.)

A few examples of tests that are run can be found in `tests`. For instance, running

``
    ./lpsolver dantzig ../tests/slide.mps
``

should give you back the maximum of 17, at the vector (1, 2, 0). The order of
the results printed out is the order in which the variables appear in the MPS
file.

You may find a few tests available in `/src/tests`. Most tests will likely hang
the simplex solver, mostly because of the lack of support for sparse matrices
in our current implementation. Smaller tests, such as `afiro.mps`, `small.mps`
and `slide.mps` will work, though. There are also some imprecisions in computation,
and that is likely due to FLENS problems. (In the end, I think I should not have
used FLENS and just stuck with Numpy or something simpler, but oh well...)

## Installing FLENS

All you have to do to install FLENS is to do

``
git clone git://github.com/michael-lehn/FLENS.git
``

**inside the `src` folder**.

## Benchmarks

The benchmarks present in the paper were performed using GLPK and lpsolve.
Both of these tools are freely available online in the links below:

[lpsolve](http://lpsolve.sourceforge.net/5.5/)

[glpk](https://www.gnu.org/software/glpk/)

On a Mac, you can easily install these via `brew` commands.
