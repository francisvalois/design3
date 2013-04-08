rm -rf reportsLeaks/
mkdir reportsLeaks/
#valgrind --leak-check=full ./devel/lib/basestation/baseStationTest > reportsLeaks/reportBaseStation.txt 2>&1
valgrind --leak-check=full -v ./devel/lib/kinocto/kinoctoTest > reportsLeaks/reportKinocto.txt 2>&1
#valgrind --leak-check=full ./devel/lib/kinocto/kinoctoLongTest > reportsLeaks/reportLongTest.txt 2>&1

