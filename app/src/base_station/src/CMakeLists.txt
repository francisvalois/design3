set(base_station_SRCS
    main.cpp
    MainWindow.cpp
)

qt4_wrap_cpp(base_station_SRCS MainWindow.h)
add_executable(base_station ${base_station_SRCS})
target_link_libraries(base_station ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

