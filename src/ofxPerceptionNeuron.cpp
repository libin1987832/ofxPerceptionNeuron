//
//  Created by Yuya Hanai, https://github.com/hanasaan/
//
#include "ofxPerceptionNeuron.h"

//#define __OS_XUN__
#define NEURONDATAREADER_EXPORTS

#include "NeuronDataReader.h"
#include "ofxBvhMod.h"
#include "BvhTemplate.h"


#define HI5GLOVE_STATIC_DEFINE
#include "Hi5Glove/Hi5Glove_C.h"


#include <stdio.h>
#include <Winsock2.h>
namespace ofxPerceptionNeuron
{
    struct BvhData
    {
        int avater_index = 0;
        string avater_name;
        bool with_disp = false;
        bool with_ref = false;
        vector<float> raw_data;
    };
	

	static std::vector<std::vector<float>> dataRecord;


	static int getFrameNum()
	{
		return frameNum;
	}
	static int getFrameIndex()
	{
		return frameindex;
	}
#pragma mark - DataReader::Impl
    class DataReader::Impl
    {
    public:
        SOCKET_REF sock = nullptr;
        
        FrameDataReceived f;
        SocketStatusChanged s;
        
        std::mutex data_lock;
        
        struct SwappableBvhData
        {
            BvhData back;
            BvhData front;
            void swap() {
                std::swap(back, front);
            }
            bool newdata = false;
            shared_ptr<ofxBvh> bvh;
            
            SwappableBvhData() {
                bvh = make_shared<ofxBvh>(bvh_header_template);
            }
            
            void update() {
                bvh->update(front.raw_data);
            }
            
            void draw() {
                bvh->draw();
            }
        };
        
        map<uint32_t, SwappableBvhData> data;
        bool newframe = false;
        uint64_t lastframe = 0;

    public:
        Impl()
        {
			status = 2; 
			frameNum = 0;
			frameindex = 0;
            f = FrameDataReceived(frameDataReceived);
            if (f) {
                BRRegisterFrameDataCallback(this, f);
            }

            s = SocketStatusChanged(socketStatusChanged);
            if (s) {
                BRRegisterSocketStatusCallback(this, s);
            }
			if (status == 2 )
			{
				ifstream myfile("H:\\out.txt");
				myfile >> frameNum;
				for (int j = 0; j < frameNum; j++)
				{
					vector<float> data(59 * 6);
					for (int i = 0; i < 59 * 6; i++)
					{
						myfile >> data[i];
					}
					dataRecord.push_back(data);
				}
			}
        }
        
        ~Impl()
        {
			cout << "asdf" << endl;
            disconnect();
			HI5Device::hi5DongleStop();
			if (status == 1)
			{
				ofstream outfile("H:\\out.txt");
				outfile << dataRecord.size()<<endl;
				for (int i = 0; i < dataRecord.size(); i++)
				{
					for (int j = 0; j < dataRecord[i].size(); j++)
					{
						outfile << dataRecord[i][j] << " ";
					}
					outfile << endl;
				}
				outfile.close();
			}
        }

		static void recvGloveHI5(float* dataBvh, vector<float>& dataOld)
		{
			using namespace HI5Device;
			GloveBVHData gloveData;
			int igNum = 2;
			for (int i = 0; i < 2&& dataOld.size()==59*6; i++)
			{
				
				int baseNum = (i == 0) ? 15 : 38;
				for (int j = baseNum+ igNum; j < baseNum + 21; j++)
				{
					dataBvh[j*6+3] = dataOld[j * 6 + 3];
					dataBvh[j * 6 + 4] = dataOld[j * 6 + 4];
					dataBvh[j * 6 + 5] = dataOld[j * 6 + 5];
				}

			}

			for(int i=0;i<2;i++)
			{
				readBVHData(&gloveData);
				if (gloveData.bUpdate)
				{
					bool withDisp = (gloveData.bWithDisp == 1);
					float* data = &gloveData.data[0];
					GloveMod glove = gloveData.glove;
					int num = 0;
					switch (glove)
					{
					case GloveMod::GM_LeftGlove:
						for (int j = igNum; j < 21; j++)
						{
							num = (withDisp ? (3 + 6 * j) : (3 + 3 * j));
							int bvhNum = num + 38 * 6;
							dataBvh[bvhNum] = data[num];
							dataBvh[bvhNum + 1] = 0.0f - data[num + 1];
							dataBvh[bvhNum + 2] = 0.0f - data[num + 2];
						}
						break;
					case GloveMod::GM_RightGlove:
						for (int i = igNum; i < 21; i++)
						{
							num = (withDisp ? (3 + 6 * i) : (3 + 3 * i));
							int bvhNum = num + 15 * 6;
							dataBvh[bvhNum] = data[num];
							dataBvh[bvhNum + 1] = 0.0f - data[num + 1];
							dataBvh[bvhNum + 2] = 0.0f - data[num + 2];
						}
						break;
					}
				}
			}
		}

		static void recvTCP(float* dataBvh, vector<float>& dataOld)
		{
			WORD wVersionRequested;
			WSADATA wsaData;
			int err;

			wVersionRequested = MAKEWORD(1, 1);

			err = WSAStartup(wVersionRequested, &wsaData);
			if (err != 0) {
				return;
			}

			if (LOBYTE(wsaData.wVersion) != 1 ||
				HIBYTE(wsaData.wVersion) != 1) {
				WSACleanup();
				return;
			}
			SOCKET sockClient = socket(AF_INET, SOCK_STREAM, 0);

			SOCKADDR_IN addrSrv;
			addrSrv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
			addrSrv.sin_family = AF_INET;
			addrSrv.sin_port = htons(6000);
			connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
			char BoneData[2000];
			int ret = recv(sockClient, BoneData, 2000, 0);
			vector<float> BoneF;
			char tmp[20] = { 0 };
			for (int i = 0, j = 0,k = 0; i < ret; i++)
			{
				
				if (BoneData[i] == ' '&&strlen(tmp)>0)
				{
					BoneF.push_back(atof(tmp));
					k = 0;
					for (int y = 0; y < 20; y++)
						tmp[y] = 0;
				}
				else 
				{
					tmp[k++] = BoneData[i];
				}
	/*			if (BoneData[i] == '(')
				{
					j = i + 1;
				}
				if (BoneData[i] == ')')
				{
					char tmp[20] = { 0 };
					for (int k = j, z = 0; k < i + 1; k++)
					{

						if (BoneData[k] == ',' || BoneData[k] == ')')
						{
							BoneF.push_back(atof(tmp));
							z = 0;
							for (int y = 0; y < 20; y++)
								tmp[y] = 0;
						}
						else
						{
							tmp[z++] = BoneData[k];
						}

					}
				}*/
			}
			if (strlen(tmp) > 0)
				BoneF.push_back(atof(tmp));
			assert(BoneF.size() == 102);
			int igNum = 1;
			for (int i = 0; i < 2; i++)
			{
				int baseNum = (i == 0) ? 15 + 1 : 38 + 1;
				for (int j = 0; j < 6; j++)
				{
					dataBvh[baseNum * 6 + j] = BoneF[i * 51 + j];
				}
				for (int j = 6, k = 0; j < 6 + 3 * 3; j = j + 3, k++)
				{
					dataBvh[(baseNum + 1) * 6 + k * 6 + 3] = BoneF[i * 51 + j];
					dataBvh[(baseNum + 1) * 6 + k * 6 + 4] = BoneF[i * 51 + j + 1];
					dataBvh[(baseNum + 1) * 6 + k * 6 + 5] = BoneF[i * 51 + j + 2];
				}
				for (int j = 6 + 3 * 3,z=0; j < 6 + 3 * 3 + 4 * 9; j = j + 9,z++)
				{
					for (int k = 0; k < 3; k++)
					{
						dataBvh[(baseNum + 4) * 6 + (k + 1) * 6 + z * 12 + 3] = BoneF[i * 51 + j + k * 3];
						dataBvh[(baseNum + 4) * 6 + (k + 1) * 6 + z * 12 + 4] = BoneF[i * 51 + j + k * 3 + 1];
						dataBvh[(baseNum + 4) * 6 + (k + 1) * 6 + z * 12 + 5] = BoneF[i * 51 + j + k * 3 + 2];
					}

				}
			}
			//for (int i = 0; i < 42; i=i+3)
			//{
			//	printf("(%f,%f,%f)@", BoneF[i], BoneF[i+1], BoneF[i+2]);
			//}
			closesocket(sockClient);
			WSACleanup();
		}
		static void recvTCPAll(float* dataBvh, vector<float>& dataOld)
		{
			WORD wVersionRequested;
			WSADATA wsaData;
			int err;

			wVersionRequested = MAKEWORD(1, 1);

			err = WSAStartup(wVersionRequested, &wsaData);
			if (err != 0) {
				return;
			}

			if (LOBYTE(wsaData.wVersion) != 1 ||
				HIBYTE(wsaData.wVersion) != 1) {
				WSACleanup();
				return;
			}
			SOCKET sockClient = socket(AF_INET, SOCK_STREAM, 0);

			SOCKADDR_IN addrSrv;
			addrSrv.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
			addrSrv.sin_family = AF_INET;
			addrSrv.sin_port = htons(6000);
			connect(sockClient, (SOCKADDR*)&addrSrv, sizeof(SOCKADDR));
			char BoneData[2000];
			int ret = recv(sockClient, BoneData, 2000, 0);
			vector<float> BoneF;
			char tmp[20] = { 0 };
			for (int i = 0, j = 0, k = 0; i < ret; i++)
			{

				if (BoneData[i] == ' '&&strlen(tmp) > 0)
				{
					BoneF.push_back(atof(tmp));
					k = 0;
					for (int y = 0; y < 20; y++)
						tmp[y] = 0;
				}
				else
				{
					tmp[k++] = BoneData[i];
				}
			}
			if (strlen(tmp) > 0)
				BoneF.push_back(atof(tmp));
			assert(BoneF.size() == 59*6);
			for (int i = 0; i < 59 * 6; i++)
				dataBvh[i] = BoneF[i];
			closesocket(sockClient);
			WSACleanup();
		}
        static void frameDataReceived(void * customObject, SOCKET_REF sockRef, BvhDataHeader * header, float * data)
        {
            Impl* self = reinterpret_cast<Impl*>(customObject);
            
            self->data_lock.lock();
            SwappableBvhData& d = self->data[header->AvatarIndex];
            BvhData& b = d.back;
            b.avater_index = header->AvatarIndex;
            b.avater_name = (string)((const char*)header->AvatarName);
            b.with_disp = header->WithDisp;
            b.with_ref = header->WithReference;
            d.newdata = true;

			vector<float> dataOld;
			dataOld.swap(b.raw_data);
            b.raw_data.clear();
			if (status == 0)
			{
				recvGloveHI5(data, dataOld);
				b.raw_data.insert(b.raw_data.end(), data, data + header->DataCount);
			}
			if (status == 1)
			{
				dataRecord.push_back(dataOld);
				//recvGloveHI5(data, dataOld);
				b.raw_data.insert(b.raw_data.end(), data, data + header->DataCount);
			}
			if (status == 2)
			{
				if (frameindex == frameNum)
					frameindex = 0;
				b.raw_data = dataRecord[frameindex++];
			}
            self->data_lock.unlock();
        }
        
        static void socketStatusChanged(void * customObject, SOCKET_REF sockeRef, SocketStatus status, char * message)
        {
            ofLogVerbose("ofxPerceptionNeuron") << message << endl;
        }

        
        void BRconnect(string ip, int port)
        {
            if (sock == nullptr) {
                sock = BRConnectTo(const_cast<char*>(ip.c_str()), port);
				HI5Device::hi5DongleStart();
				HI5Device::setReducedOutputDataFreq(1);
            }
			
        }
        
        bool isConnected() const {
            if (sock == nullptr) {
                return false;
            }
            SocketStatus ss = BRGetSocketStatus(sock);
            return ss == CS_Running;
        }
        
        void disconnect()
        {
            if (sock != nullptr) {
                BRCloseSocket(sock);
                sock = nullptr;
            }
        }
        
        void update()
        {
			BvhDataHeader  header;
			 header.AvatarIndex=0;
			header.AvatarName[0]=42;
			header.WithDisp=0;
			header.WithReference=0;
			frameDataReceived(this, nullptr, &header, nullptr);
            uint64_t frame = ofGetFrameNum();
            if (frame != lastframe) {
                newframe = false;
                lastframe = frame;
            }
            data_lock.lock();
            for (auto& p : data) {
                if (p.second.newdata) {
                    p.second.swap();
                    p.second.update();
                    p.second.newdata = false;
                    newframe = true;
                }
            }
            data_lock.unlock();
        }
        
        void draw()
        {
            for (auto& p : data) {
                p.second.draw();
            }
        }
        
        bool isFrameNew() const {
            return newframe;
        }
    };
    
#pragma mark - Skeleton
    void Skeleton::debugDraw() const
    {
        ofVec3f vn;
        ofPushStyle();
        ofNoFill();
        for (auto & p : joints) {
            ofPushMatrix();
            ofMultMatrix(p.global_transform);
            if (p.name.find("Hand") == string::npos &&
                p.name.find("Site") == string::npos) {
                //ofDrawBox(10);
                //ofDrawAxis(15);
            } else {
                //ofDrawBox(1);
                ofDrawAxis(1.5);
            }
            for (auto & q : p.children) {
                ofVec3f& v = q->offset;
                ofDrawLine(vn, v);
            }
            ofPopMatrix();
            
        }
        ofPopStyle();
    }
    
#pragma mark - DataReader
    DataReader::DataReader()
    {
        impl = make_shared<Impl>();
    }
    
    void DataReader::BRconnect(string ip, int port)
    {
        impl->BRconnect(ip, port);
    }
    
    bool DataReader::isConnected() const
    {
        return impl->isConnected();
    }
    
    bool DataReader::isFrameNew() const
    {
        return impl->isFrameNew();
    }
    
    void DataReader::update()
    {
        impl->update();
        
        // copy
        if (skeletons.size() != impl->data.size()) {
            skeletons.resize(impl->data.size());
        }
        for (int i=0; i<skeletons.size(); ++i) {
            auto & bvh = impl->data[i].bvh;
            auto & s = skeletons[i];
            skeletons_map[impl->data[i].front.avater_name] = &s;
            if (s.joints.size() != bvh->getNumJoints()) {
                s.name = impl->data[i].front.avater_name;
                s.joints.resize(bvh->getNumJoints());
                for (int j=0; j<s.joints.size(); ++j) {
                    auto* bvhj = bvh->getJoint(j);
                    auto& sj = s.joints[j];
                    sj.name = bvhj->getName();
                    s.joints_map[bvhj->getName()] = &sj;

                }
                for (int j=0; j<s.joints.size(); ++j) {
                    auto* bvhj = bvh->getJoint(j);
                    auto& sj = s.joints[j];
                    if (bvhj->getParent()) {
                        string pname = bvhj->getParent()->getName();
                        sj.parent = s.joints_map[pname];
                    }
                    sj.children.clear();
                    for (auto& c : bvhj->getChildren()) {
                        if (c) {
                            string cname = c->getName();
                            if (s.joints_map.find(cname) != s.joints_map.end()) {
                                sj.children.push_back(s.joints_map[cname]);
                            }
                        }
                    }
                }
            }
            for (int j=0; j<s.joints.size(); ++j) {
                auto* bvhj = bvh->getJoint(j);
                auto& sj = s.joints[j];
                sj.global_transform = bvhj->getGlobalMatrix();
                sj.transform = bvhj->getMatrix();
                sj.offset = bvhj->getOffset();
            }
        }
    }
    
    void DataReader::disconnect()
    {
        impl->disconnect();
    }
    
    void DataReader::debugDraw() const
    {
//        impl->draw();
        for (auto & p : skeletons) {
            p.debugDraw();
        }
    }

    const Skeleton& DataReader::getSkeletonByName(string name) const
    {
        const auto& it = skeletons_map.find(name);
        if (it != skeletons_map.end()) {
            return *it->second;
        }
    }

}