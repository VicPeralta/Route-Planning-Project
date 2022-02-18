#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{
	std::ifstream is{ path, std::ios::binary | std::ios::ate };
	if (!is)
		return std::nullopt;

	auto size = is.tellg();
	std::vector<std::byte> contents(static_cast<std::size_t>(size));

	is.seekg(0);
	is.read((char*)contents.data(), size);

	if (contents.empty())
		return std::nullopt;
	return std::move(contents);
}

int main(int argc, const char **argv)
{
	std::string osm_data_file = "";
	if (argc > 1) {
		for (int i = 1; i < argc; ++i)
			if (std::string_view{ argv[i] } == "-f" && ++i < argc)
				osm_data_file = argv[i];
	}
	else {
		std::cout << "To specify a map file use the following format: " << std::endl;
		std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
		osm_data_file = "../map.osm";
	}

	std::vector<std::byte> osm_data;

	if (osm_data.empty() && !osm_data_file.empty()) {
		std::cout << "Reading OpenStreetMap data from the following file: " << osm_data_file << std::endl;
		auto data = ReadFile(osm_data_file);
		if (!data)
			std::cout << "Failed to read." << std::endl;
		else
			osm_data = std::move(*data);
	}

	// TODO 1: Declare floats `start_x`, `start_y`, `end_x`, and `end_y` and get
	// user input for these values using std::cin. Pass the user input to the
	// RoutePlanner object below in place of 10, 10, 90, 90.
	float start_x{ 0 }, start_y{ 0 };
	float end_x{ 0 }, end_y{ 0 };
	std::cout << "Please enter the starting points x and y\nValid values are from 0 to 100\n";
	std::cout << "Example: 0 0\n";
	std::cin >> start_x;
	std::cin >> start_y;
	if ((start_x < 0 || start_y < 0) || (start_x > 100 || start_y > 100))
	{
		std::cout << "The values for the starting point must be between 0 and 100\n";
		return -1;
	}
	std::cout << "Please enter the end points x and y\nValid values are from 0 to 100\n";
	std::cout << "Example: 90 90\n";
	std::cin >> end_x;
	std::cin >> end_y;
	
	
	if ((end_x < 0 || end_y < 0) || (end_x > 100 || end_y > 100))
	{
		std::cout << "The values for the end point must be between 0 and 100\n";
		return -2;
	}
	std::cout << "Building model for:" << start_x << " " << start_y << " "
		<< " " << end_x << " " << end_y << "\n";
	// Build Model.
	RouteModel model{ osm_data };
	/// for debugging purposes
	/*for (float x = 0; x <= 100; x++)
	{
		for (float y = 0; y <= 100; y++)
		{
			RouteModel model{ osm_data };
			RoutePlanner route_planner{ model, x, y, 100, 100 };
			std::cout << "Building model for: " << x << " " << y << " 100 100 \n";
			route_planner.AStarSearch();
			auto d = route_planner.GetDistance();
			if(d==0)
				std::cout << "Distance:" << d << "\n";
		}
	}*/
	// Create RoutePlanner object and perform A* search.
	RoutePlanner route_planner{ model, start_x, start_y, end_x, end_y };
	route_planner.AStarSearch();
	auto distance = route_planner.GetDistance();
	if (distance>0) {
		std::cout << "Distance: " << distance << " meters. \n";
		// Render results of search.
		Render render{ model }; 
		auto display = io2d::output_surface{ 400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30 };
		display.size_change_callback([](io2d::output_surface& surface) {
			surface.dimensions(surface.display_dimensions());
		});
		display.draw_callback([&](io2d::output_surface& surface) {
			render.Display(surface);
		});
		display.begin_show();
		return 0;
	}
	std::cout << "Could not find a route!\n";
}
