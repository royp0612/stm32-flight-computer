CC = gcc
CFLAGS ?= -std=c11 -Wall -Wextra -Werror

TEST_BUILD_DIR := tests/build
TEST_INCLUDES := -ICore/Inc -Itests/unity -Itests/mocks
TEST_SOURCES := tests/mocks/mock_spi_bus.c tests/unity/unity.c
TEST_HEADERS := Core/Inc/spi_bus.h tests/mocks/mock_spi_bus.h \
                tests/unity/unity.h tests/unity/unity_internals.h
PLATFORM_SOURCES := tests/mocks/mock_platform.c
PLATFORM_HEADERS := Core/Inc/platform.h tests/mocks/mock_platform.h

ifeq ($(OS),Windows_NT)
EXEEXT := .exe
endif

IMU_TEST := $(TEST_BUILD_DIR)/test_imu$(EXEEXT)
BARO_TEST := $(TEST_BUILD_DIR)/test_baro$(EXEEXT)
FLASH_TEST := $(TEST_BUILD_DIR)/test_flash$(EXEEXT)

.PHONY: test
test: $(IMU_TEST) $(BARO_TEST) $(FLASH_TEST)
	"$(abspath $(IMU_TEST))"
	"$(abspath $(BARO_TEST))"
	"$(abspath $(FLASH_TEST))"

$(IMU_TEST): Core/Src/imu.c tests/test_imu.c $(TEST_SOURCES) \
             Core/Inc/imu.h $(TEST_HEADERS) | $(TEST_BUILD_DIR)
	$(CC) $(CPPFLAGS) $(TEST_INCLUDES) $(CFLAGS) $(LDFLAGS) $(filter %.c,$^) $(LDLIBS) -o "$@"

$(BARO_TEST): Core/Src/baro.c tests/test_baro.c $(TEST_SOURCES) $(PLATFORM_SOURCES) \
              Core/Inc/baro.h $(TEST_HEADERS) $(PLATFORM_HEADERS) | $(TEST_BUILD_DIR)
	$(CC) $(CPPFLAGS) $(TEST_INCLUDES) $(CFLAGS) $(LDFLAGS) $(filter %.c,$^) $(LDLIBS) -lm -o "$@"

$(FLASH_TEST): Core/Src/flash.c tests/test_flash.c $(TEST_SOURCES) $(PLATFORM_SOURCES) \
               Core/Inc/flash.h $(TEST_HEADERS) $(PLATFORM_HEADERS) | $(TEST_BUILD_DIR)
	$(CC) $(CPPFLAGS) $(TEST_INCLUDES) $(CFLAGS) $(LDFLAGS) $(filter %.c,$^) $(LDLIBS) -o "$@"

$(TEST_BUILD_DIR):
ifeq ($(OS),Windows_NT)
	cmd.exe /c mkdir "$(subst /,\,$@)"
else
	mkdir -p "$@"
endif
