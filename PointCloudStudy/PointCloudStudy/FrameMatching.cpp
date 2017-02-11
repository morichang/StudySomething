#include "FrameMatching.h"

FrameMatching::FrameMatching()
{
	char szUserName[256] = { '\0' };
	DWORD dwSize = sizeof(szUserName) / sizeof(szUserName[0]);
	GetUserName(szUserName, &dwSize);
	userName = std::string(szUserName);

	//dind_id =
	//{
	//	"DIND104", "DIND107", "DIND108", "DIND109", "DIND110", "DIND111", "DIND112"
	//};
	date = "2017-01-31T21-21-";
}

inline void FrameMatching::split(std::vector<std::string> &v, const std::string &input_string, const std::string &delimiter)
{
	std::string::size_type index = input_string.find_first_of(delimiter);

	if (index != std::string::npos) {
		v.push_back(input_string.substr(0, index));
		split(v, input_string.substr(index + 1), delimiter);
	}
	else {
		v.push_back(input_string);
	}
}

void FrameMatching::loadExtractFrameList(std::vector<std::string> &dind_id)
{
	std::string str;
	std::vector<std::string> save_date;  // ãLò^ÇµÇΩì˙ït
	std::vector<std::string> save_TOD;  // ãLò^ÇµÇΩéûçè(Time Of Day)

	std::ifstream ifs_extract_data;

	for (int i = 0; i < dind_id.size(); ++i) {
		load_path = "C:/Users/" + userName + "/Desktop/" + dind_id[i] + "-PC/2017-01-31_21-20-00_" + dind_id[i] + "-PC";
		ifs_extract_data.open(load_path + "_extract.txt", std::ios::in);
		while (!ifs_extract_data.eof()) {
			getline(ifs_extract_data, str);
			split(save_date, str, "T");
			split(save_TOD, save_date[1], "-");
			if (dind_id[i] == "DIND104") {
				dind104List.push_back(save_TOD[2]);
			}
			else if (dind_id[i] == "DIND107") {
				dind107List.push_back(save_TOD[2]);
			}
			else if (dind_id[i] == "DIND108") {
				dind108List.push_back(save_TOD[2]);
			}
			else if (dind_id[i] == "DIND109") {
				dind109List.push_back(save_TOD[2]);
			}
			else if (dind_id[i] == "DIND110") {
				dind110List.push_back(save_TOD[2]);
			}
			else if (dind_id[i] == "DIND111") {
				dind111List.push_back(save_TOD[2]);
			}
			else if (dind_id[i] == "DIND112") {
				dind112List.push_back(save_TOD[2]);
			}
			else {
				return;
			}
			save_date.clear();
			save_TOD.clear();
		}
		ifs_extract_data.close();
		std::cout << "íäèo" << std::endl;
	}

	return;
}

void FrameMatching::extractFrameMatching(std::vector<std::string> &dind)
{
	const double th = 33333.33;
	std::string str_list1;
	std::string str_list2;
	std::vector<std::string> list1;
	std::vector<std::string> list2;
	std::vector<std::string> base_second;
	std::vector<std::string> compair11_sec;

	//loadExtractFrameList(dind);

	//std::ofstream ofs("dind112to108List.txt", std::ios::out);
	//for (int i = 0; i < dind112List.size(); ++i) {
	//	split(base_second, dind112List[i], "_");
	//	// DIND104
	//	for (int comp_i = 0; comp_i < dind108List.size(); ++comp_i) {
	//		split(compair11_sec, dind108List[comp_i], "_");
	//		if ((base_second[0] == compair11_sec[0])) {
	//			if (std::fabs(std::stod(base_second[1]) - std::stod(compair11_sec[1])) < th) {
	//				ofs	<< date << dind108List[comp_i] << "," << date << dind112List[i] << std::endl;
	//			}
	//		}
	//		compair11_sec.clear();
	//	}
	//	base_second.clear();
	//}
	//ofs.close();

	std::ifstream ifs_list1("DIND112_104_107_108_109_110_pair.txt", std::ios::in);
	if (ifs_list1.fail()) {
		std::cerr << "é∏îs" << std::endl;
		return;
	}

	std::ofstream ofs_pair("DIND112_104_107_108_109_110_111_pair.txt", std::ios::out);
	if (ofs_pair.fail()) {
		std::cerr << "é∏îs" << std::endl;
		return;
	}

	while (!ifs_list1.eof()) {
		getline(ifs_list1, str_list1);
		split(list1, str_list1, ",");
		std::ifstream ifs_list2("dind112to111List.txt", std::ios::in);
		if (ifs_list2.fail()) {
			std::cerr << "é∏îs" << std::endl;
			return;
		}
		while (!ifs_list2.eof()) {
			getline(ifs_list2, str_list2);
			split(list2, str_list2, ",");
			if (list2[1] == list1[5]) {
				ofs_pair << list1[0] << "," << list1[1] << "," << list1[2] << "," << list1[3] << "," << list1[4] << "," << str_list2 << std::endl;
			}
			list2.clear();
		}
		list1.clear();
	}

	return;
}