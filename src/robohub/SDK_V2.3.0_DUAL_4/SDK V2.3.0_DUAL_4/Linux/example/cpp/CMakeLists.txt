set(CPP_EXAMPLE_SOURCES
    # "01.login.cpp"
    # "02.robot_status.cpp"
    # "03.analog_output.cpp"
    # "04.robotstatus_dout.cpp"
    # "05.nonblock_jointmove.cpp"
    # "07.test_get_tcppos.cpp"
    # "08.test_reconnect_robot.cpp"
    # "09.test_get_tool_id.cpp"
    # "10.is_in_pos.cpp"
    # "11.test_get_tcp.cpp"
    # "12.ftp.cpp"
    # "13.set_sdk_log_path.cpp"
    # "14.lineare_move.cpp"
    # "15.tio_pin_mode.cpp"
    # "16.rs485_chn_mode.cpp"
    # "17.test_joint_move.cpp"
    # "18.test_get_program_state.cpp"
    # "19.test_servoj.cpp"
    # "20.set_user_frame.cpp"
    # "21.test_movc.cpp"
    # "22.add_del_tio_signal.cpp"
    "23.multi_movj.cpp"
    "24.multi_movl.cpp"
    "25.kine.cpp"
    "26.motion_abort.cpp"
    "27.payload.cpp"
    "28.fct.cpp"
    "28.drag.cpp"
    "30.edgservo.cpp"
    "31.edgservo_fct.cpp"
    "32.controlloop.cpp"
    "40.error_and_clear.cpp"
    "41.collision.cpp"
    "42.enable.cpp"
    "43.tool_offset.cpp"
    "44.base_offset.cpp"
    "45.network_exception_handlet.cpp"
    "46.set_tio.cpp"
    "47.read_tio_param.cpp"
)

foreach(src ${CPP_EXAMPLE_SOURCES})
    # compile
    set(TMP_TGT_NAME "${src}.out")
    link_directories(${CMAKE_SOURCE_DIR}/out/shared)
    link_directories(${CMAKE_SOURCE_DIR}/out/shared/${CMAKE_BUILD_TYPE})
    add_executable(${TMP_TGT_NAME} ${src} common.h timespec.cpp)
    target_include_directories(${TMP_TGT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/inc)
    # target_link_directories(${TMP_TGT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/out/shared)
    if (WIN32)
        target_link_libraries(${TMP_TGT_NAME} PRIVATE jakaAPI)
    else()
        target_link_libraries(${TMP_TGT_NAME} PRIVATE ${SDK_NAME} pthread)
    endif()
    # install
    install(
        FILES ${src}
        DESTINATION ${JAKA_SDK_EXAMPLE_INSTALL_DIR}/c++
    )
    add_dependencies(${TMP_TGT_NAME} ${SDK_SHARED_TARGET})
endforeach()

install(
    FILES common.h
    DESTINATION ${JAKA_SDK_EXAMPLE_INSTALL_DIR}/c++
)

unset(CPP_EXAMPLE_SOURCES)
