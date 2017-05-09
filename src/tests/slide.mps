NAME          SlideProb
ROWS
 N  COST
 L  R01
 L  R02
 L  R03
COLUMNS
    X01       R01                  1   R02                  1
    X01       R03                  1   COST                 5
    X02       R01                  1   R02                  2
    X02       R03                  1   COST                 6
    X03       R01                  1   R02                  3
    X03       R03                  2   COST                 9
    X04       R01                  1   R02                  1
    X04       R03                  3   COST                 8
RHS
    B         R01                  6   R02                  5
    B         R03                  3
BOUNDS
 LO BOUN      X01                  0
 LO BOUN      X02                  0
 LO BOUN      X03                  0
 LO BOUN      X04                  0
ENDATA
