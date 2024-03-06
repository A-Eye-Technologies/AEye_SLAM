#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include <grpc/grpc.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include <vslam.grpc.pb.h>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::Status;
using vslam::Percept;
using vslam::Response;
using vslam::Frame;
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
    Status ProcessPercepts(ServerContext *context, ServerReader<Percept> *reader, Response *response) override
    {
        Percept percept;
        while (reader->Read(&percept))
        {
            const Frame &frame = percept.frame();

            google::protobuf::Timestamp timestamp = frame.timestamp();
            double tframe = timestamp.seconds() + timestamp.nanos() / 1e9;

            cv::Mat rgb_image = decodeImage(frame.rgb_image(),cv::IMREAD_COLOR);
            cv::Mat depth_image = decodeImage(frame.depth_image(), cv::IMREAD_ANYDEPTH);
        }

        return Status::OK;
    }
};

void RunServer()
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
    RunServer();

    return 0;
}