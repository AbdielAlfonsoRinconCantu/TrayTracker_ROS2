#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"  // Add this includ

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("penny_tray_constructor")
    {
    
        publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
            "penny_grid", 10);

        auto topic_callback = [this](std_msgs::msg::Int32MultiArray::UniquePtr msg) {
            data_.insert(data_.end(), msg->data.begin(), msg->data.end());
            RCLCPP_INFO(this->get_logger(), "Accumulated size: %zu", data_.size());

            for (size_t i = 0; i < data_.size(); ++i) {
                if (data_[i] == 0 && (i + 1) < data_.size()) {
                    int penny_count = data_[i + 1];
                    RCLCPP_INFO(this->get_logger(), "New tray with %d pieces", penny_count);

                    // 2) Extract all x,y pairs
                    std::vector<int> x_coords;
                    std::vector<int> y_coords;
                    x_coords.reserve(penny_count);
                    y_coords.reserve(penny_count);

                    for (int j = 2; j < 2 + 2 * penny_count; j += 2) {
                        if (static_cast<size_t>(j + 1) < data_.size()) {
                            x_coords.push_back(data_[j]);
                            y_coords.push_back(data_[j + 1]);
                        }
                    }

                    // Print all coordinates
                    std::cout << "Coordinates:\n";
                    for (int j = 0; j < penny_count; ++j) {
                        std::cout << "(" << x_coords[j] << ", " << y_coords[j] << ")\n";
                    }

                    // 3) Compute min/max
                    auto [min_x_it, max_x_it] = std::minmax_element(x_coords.begin(), x_coords.end());
                    auto [min_y_it, max_y_it] = std::minmax_element(y_coords.begin(), y_coords.end());
                    int min_x = *min_x_it, max_x = *max_x_it;
                    int min_y = *min_y_it, max_y = *max_y_it;

                    // 4) Compute cell size for 13 bins
                    double cell_width = (max_x - min_x) / 13.0;
                    double cell_height = (max_y - min_y) / 13.0;

                    // 5) Create 13x13 grid of 0s, fill it
                    static const int GRID_SIZE = 13;
                    std::vector<std::vector<int>> grid(GRID_SIZE, std::vector<int>(GRID_SIZE, 0));

                    for (int j = 0; j < penny_count; ++j) {
                        double x = static_cast<double>(x_coords[j]);
                        double y = static_cast<double>(y_coords[j]);
                        int col = static_cast<int>(std::floor((x - min_x) / cell_width));
                        int row = static_cast<int>(std::floor((y - min_y) / cell_height));
                        if (col < 0) col = 0; if (col >= GRID_SIZE) col = GRID_SIZE - 1;
                        if (row < 0) row = 0; if (row >= GRID_SIZE) row = GRID_SIZE - 1;
                        grid[row][col] = 1;
                    }

                    // 6) Print the 13x13 grid
                    std::cout << "Grid presence:\n";
                    for (int row = 0; row < GRID_SIZE; ++row) {
                        for (int col = 0; col < GRID_SIZE; ++col) {
                            std::cout << grid[row][col] << " ";
                        }
                        std::cout << "\n";
                    }

                    // After creating the grid, publish it
                    auto grid_msg = std_msgs::msg::UInt8MultiArray();
                    grid_msg.data.reserve(GRID_SIZE * GRID_SIZE);
                    
                    // Flatten the 2D grid into 1D array
                    for (int row = 0; row < GRID_SIZE; ++row) {
                        for (int col = 0; col < GRID_SIZE; ++col) {
                            grid_msg.data.push_back(static_cast<uint8_t>(grid[row][col]));
                        }
                    }
                    
                    // Publish the grid
                    publisher_->publish(grid_msg);
                    RCLCPP_INFO(this->get_logger(), "Published grid data");
                }
            }
        };

        subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "coordinates_pennies", 10, topic_callback
        );
    }

private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;  // Add this member
    std::vector<int> data_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}

