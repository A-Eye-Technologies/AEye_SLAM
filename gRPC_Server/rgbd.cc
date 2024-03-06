#include <iostream>
#include <string>
#include <algorithm>
#include <chrono>
#include <opencv2/opencv.hpp>

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include <vslam.grpc.pb.h>

#include <System.h>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::Status;
using vslam::Frame;
using vslam::Percept;
using vslam::Response;
using vslam::Vslam;

cv::Mat decodeImage(const std::string &image_bytes, int decode_type)
{
    // Decode the PNG image from the byte stream
    std::vector<uchar> buffer(image_bytes.begin(), image_bytes.end());
    cv::Mat decoded_image = cv::imdecode(buffer, decode_type);
    return decoded_image;
}

class SlamService final : public Vslam::Service
{
public:
    explicit SlamService(const ORB_SLAM3::System &slam) : SLAM(slam) {}

    Status ProcessPercepts(ServerContext *context, ServerReader<Percept> *reader, Response *response) override
    {
        Percept percept;
        while (reader->Read(&percept))
        {
            const Frame &frame = percept.frame();

            google::protobuf::Timestamp timestamp = frame.timestamp();
            double tframe = timestamp.seconds() + timestamp.nanos() / 1e9;

            cv::Mat rgb_image = decodeImage(frame.rgb_image(), cv::IMREAD_COLOR);
            cv::Mat depth_image = decodeImage(frame.depth_image(), cv::IMREAD_ANYDEPTH);

            AdjustScale(rgb_image);
            AdjustScale(depth_image);

            // Process the current frame
            SLAM.TrackRGBD(rgb_image, depth_image, tframe);
        }

        return Status::OK;
    }

private:
    ORB_SLAM3::System SLAM;

    void AdjustScale(cv::Mat image)
    {
        float imageScale = SLAM.GetImageScale();

        if (imageScale != 1.f)
        {
            int width = image.cols * imageScale;
            int height = image.rows * imageScale;
            cv::resize(image, image, cv::Size(width, height));
        }
    }
};

void RunServer(const ORB_SLAM3::System &SLAM)
{
    std::string server_address("0.0.0.0:50051");
    SlamService service;

    ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;
    server->Wait();
}

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << std::endl
                  << "Usage: ./rgbd path_to_vocabulary path_to_configurations" << std::endl;
        return 1;
    }

    std::string vocabulary_path = argv[1];
    std::string configurations_path = argv[2];

    // Create SLAM system and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocabulary_path, configurations_path, ORB_SLAM3::System::RGBD, true);

    RunServer(SLAM);

    // Stop the ORB_SLAM algorithm
    SLAM.Shutdown();

    return 0;
}