// encoding: utf-8

#include <JAKAZuRobot.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include "common.h"

#if defined(_WIN32)
#include <windows.h>
#elif defined(__linux__)
#include <string.h>
#endif // _WIN32

bool is_path_exist(const std::string &path)
{
#if defined(_WIN32)
    DWORD fileAttributes = GetFileAttributes(path.c_str());
    return !(fileAttributes == INVALID_FILE_ATTRIBUTES);
#else
    return false;
#endif
}

#if defined(_WIN32)
/**
 * On Windows, this helper converts UTF-8 text into GBK encoding
 */
void U2G(const char *utf8, char* gbk_bytes, int32_t* gbk_bytes_num)//UTF-8 TO GBK
{
	int len = MultiByteToWideChar(CP_UTF8, 0, utf8, -1, NULL, 0);
    ASSERT_TRUE_OR_EXIT(*gbk_bytes_num >= len + 1, "gbk bytes buffer size is too small");

	wchar_t *wstr = new wchar_t[len + 1];
	memset(wstr, 0, len + 1);
	MultiByteToWideChar(CP_UTF8, 0, utf8, -1, wstr, len);
	len = WideCharToMultiByte(CP_ACP, 0, wstr, -1, NULL, 0, NULL, NULL);
	char *str = new char[len + 1];
	memset(str, 0, len + 1);
	WideCharToMultiByte(CP_ACP, 0, wstr, -1, str, len, NULL, NULL);
	memcpy(gbk_bytes, str, len + 1);
	if (wstr)
		delete[] wstr;
	if (str)
		delete[] str;

    *gbk_bytes_num = len + 1;
}
#endif

int main()
{
    const int PATH_BUF_LEN = 512;
    JAKAZuRobot robot;
    int ret;
    char path_buf_1[PATH_BUF_LEN];
    char path_buf_2[PATH_BUF_LEN];
    int32_t path_buf_len = PATH_BUF_LEN;

#if defined(_WIN32)
    char local[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\testfile";
    char local_download[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\testfile.tmp";
#elif defined(__linux__)
    char local[] = "/home/zgj/sdk_dev/jakaAPI-c/testfile";
    char local_download[] = "/home/zgj/sdk_dev/jakaAPI-c/testfile.tmp";
#endif
    char remote[] = "/log/testfile";
    char remote_new[] = "/log/testfile.tmp";

    ret = robot.login_in("192.168.2.200");
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "login.");

    ret = robot.init_ftp_client();

    // Upload file
    ret = robot.upload_file(local, remote, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "upload file");

    // Rename remote file
    ret = robot.rename_ftp_file(remote, remote_new, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "rename file");

    // Download file
    ret = robot.download_file(local_download, remote_new, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "download file");

    // Delete remote file
    ret = robot.del_ftp_file(remote_new, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "del remote file");

#if defined(_WIN32)
    char local_dir[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\test_dir";
    char local_download_dir[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\test_dir.tmp";
#elif defined(__linux__)
    char local_dir[] = "/home/zgj/sdk_dev/jakaAPI-c/test_dir/";
    char local_download_dir[] = "/home/zgj/sdk_dev/jakaAPI-c/test_dir.tmp/";
#endif
    char remote_dir[] = "log/test_dir";
    char remote_dir_new[] = "/log/test_dir.new";

    // Upload directory
    ret = robot.upload_file(local_dir, remote_dir, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "upload dir");

    // Rename remote directory
    ret = robot.rename_ftp_file(remote_dir, remote_dir_new, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "rename dir");

    // Download directory
    ret = robot.download_file(local_download_dir, remote_dir_new, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "download dir");

    // Delete remote directory
    ret = robot.del_ftp_file(remote_dir_new, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "remove remote dir");

#if defined(_WIN32)
    char local_cn[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\unicode_file";
    char local_download_cn[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\unicode_file.tmp";
#elif defined(__linux__)
    char local_cn[] = "/home/zgj/sdk_dev/jakaAPI-c/unicode_file";
    char local_download_cn[] = "/home/zgj/sdk_dev/jakaAPI-c/unicode_file.tmp";
#endif
    char remote_cn[] = "/log/unicode_file";
    char remote_new_cn[] = "/log/unicode_file.tmp";

    // Upload file
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(local_cn, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_cn, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, local_cn, sizeof(local_cn));
    memcpy(path_buf_2, remote_cn, sizeof(remote_cn));
#endif
    ret = robot.upload_file(path_buf_1, path_buf_2, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "upload cn file");

    // Rename remote file
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_cn, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_new_cn, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, remote_cn, sizeof(remote_cn));
    memcpy(path_buf_2, remote_new_cn, sizeof(remote_new_cn));
#endif
    ret = robot.rename_ftp_file(path_buf_1, path_buf_2, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "rename cn file");

    // Download file
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(local_download_cn, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_new_cn, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, local_download_cn, sizeof(local_download_cn));
    memcpy(path_buf_2, remote_new_cn, sizeof(remote_new_cn));
#endif
    ret = robot.download_file(path_buf_1, path_buf_2, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "download cn file");

    // Delete remote file
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_new_cn, path_buf_1, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, remote_new_cn, sizeof(remote_new_cn));
#endif
    ret = robot.del_ftp_file(path_buf_1, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "del remote cn file");

#if defined(_WIN32)
    char local_dir_cn[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\unicode_dir";
    char local_download_dir_cn[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\unicode_dir.tmp";
#elif defined(__linux__)
    char local_dir_cn[] = "/home/zgj/sdk_dev/jakaAPI-c/unicode_dir";
    char local_download_dir_cn[] = "/home/zgj/sdk_dev/jakaAPI-c/unicode_dir.tmp";
#endif
    char remote_dir_cn[] = "log/unicode_dir";
    char remote_dir_new_cn[] = "/log/unicode_dir.new";

    // Upload directory
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(local_dir_cn, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_cn, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, local_dir_cn, sizeof(local_dir_cn));
    memcpy(path_buf_2, remote_dir_cn, sizeof(remote_dir_cn));
#endif
    ret = robot.upload_file(path_buf_1, path_buf_2, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "upload cn dir");

    // Rename remote directory
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_cn, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_new_cn, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, remote_dir_cn, sizeof(remote_dir_cn));
    memcpy(path_buf_2, remote_dir_new_cn, sizeof(remote_dir_new_cn));   
#endif
    ret = robot.rename_ftp_file(path_buf_1, path_buf_2, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "rename cn dir");

    // Download directory
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(local_download_dir_cn, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_new_cn, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, local_download_dir_cn, sizeof(local_download_dir_cn));
    memcpy(path_buf_2, remote_dir_new_cn, sizeof(remote_dir_new_cn));   
#endif
    ret = robot.download_file(path_buf_1, path_buf_2, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "download cn dir");

    // Delete remote directory
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_new_cn, path_buf_1, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, remote_dir_new_cn, sizeof(remote_dir_new_cn));
#endif
    ret = robot.del_ftp_file(path_buf_1, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "remove remote cn dir");

#if defined(_WIN32)
    char local_dir_cn_sp[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\unicode dir";
    char local_download_dir_cn_sp[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\unicode dir.tmp";
#elif defined(__linux__)
    char local_dir_cn_sp[] = "/home/zgj/sdk_dev/jakaAPI-c/unicode dir/";
    char local_download_dir_cn_sp[] = "/home/zgj/sdk_dev/jakaAPI-c/unicode dir.tmp/";
#endif
    char remote_dir_cn_sp[] = "log/unicode dir";
    char remote_dir_new_cn_sp[] = "/log/unicode dir.new";

    // Upload directory
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(local_dir_cn_sp, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_cn_sp, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, local_dir_cn_sp, sizeof(local_dir_cn_sp));
    memcpy(path_buf_2, remote_dir_cn_sp, sizeof(remote_dir_cn_sp));
#endif
    ret = robot.upload_file(path_buf_1, path_buf_2, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "upload cn sp dir");

    // Rename remote directory
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_cn_sp, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_new_cn_sp, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, remote_dir_cn_sp, sizeof(remote_dir_cn_sp));
    memcpy(path_buf_2, remote_dir_new_cn_sp, sizeof(remote_dir_new_cn_sp));
#endif
    ret = robot.rename_ftp_file(path_buf_1, path_buf_2, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "rename cn sp dir");

    // Download directory
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(local_download_dir_cn_sp, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_new_cn_sp, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, local_download_dir_cn_sp, sizeof(local_download_dir_cn_sp));
    memcpy(path_buf_2, remote_dir_new_cn_sp, sizeof(remote_dir_new_cn_sp));
#endif
    ret = robot.download_file(path_buf_1, path_buf_2, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "download cn sp dir");

    // Delete remote directory
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_dir_new_cn_sp, path_buf_1, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, remote_dir_new_cn_sp, sizeof(remote_dir_new_cn_sp));
#endif
    ret = robot.del_ftp_file(path_buf_1, 2);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "remove remote cn sp dir");

#if defined(_WIN32)
    char local_cn_sp[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\unicode file";
    char local_download_cn_sp[] = "C:\\Users\\jenkins\\Desktop\\sdk_dev\\jakaAPI-c\\jakaAPI-c\\unicode file.tmp";
#elif defined(__linux__)
    char local_cn_sp[] = "/home/zgj/sdk_dev/jakaAPI-c/unicode file";
    char local_download_cn_sp[] = "/home/zgj/sdk_dev/jakaAPI-c/unicode file.tmp";
#endif
    char remote_cn_sp[] = "log/unicode file";
    char remote_new_cn_sp[] = "/log/unicode file.new";

    // Upload file
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(local_cn_sp, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_cn_sp, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, local_cn_sp, sizeof(local_cn_sp));
    memcpy(path_buf_2, remote_cn_sp, sizeof(remote_cn_sp));
#endif
    ret = robot.upload_file(path_buf_1, path_buf_2, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "upload cn sp file");

    // Rename remote file
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_cn_sp, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_new_cn_sp, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, remote_cn_sp, sizeof(remote_cn_sp));
    memcpy(path_buf_2, remote_new_cn_sp, sizeof(remote_new_cn_sp));
#endif
    ret = robot.rename_ftp_file(path_buf_1, path_buf_2, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "rename cn sp file");

    // Download file
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(local_download_cn_sp, path_buf_1, &path_buf_len);
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_new_cn_sp, path_buf_2, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, local_download_cn_sp, sizeof(local_download_cn_sp));
    memcpy(path_buf_2, remote_new_cn_sp, sizeof(remote_new_cn_sp));
#endif
    ret = robot.download_file(path_buf_1, path_buf_2, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "download cn sp file");

    std::string remote_get_dir = "log";
    char result_get[4096];
    ret = robot.get_ftp_dir(remote_get_dir.c_str(), 0, result_get);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "get ftp dir");
    std::cout << result_get << "\n";

    // Delete remote file
#if defined(_WIN32)
    path_buf_len = PATH_BUF_LEN;
    U2G(remote_new_cn_sp, path_buf_1, &path_buf_len);
#elif defined(__linux__)
    memcpy(path_buf_1, remote_new_cn_sp, sizeof(remote_new_cn_sp));
#endif
    ret = robot.del_ftp_file(path_buf_1, 1);
    ASSERT_TRUE_OR_LOG(ret == ERR_SUCC, "remove remote cn sp file");

    ret = robot.close_ftp_client();
    return 0;
}