#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <iostream>
#include <string>
#include <CaribouLite.hpp>
#include <thread>
#include <complex>
#include <unistd.h>
#include <cmath>
#include <memory>
#include <deque>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <nlohmann/json.hpp>
#include <string.h>
#include <unistd.h>
#include <pwd.h>

class ScanRange
{
public:
    ScanRange(float step, const CaribouLiteFreqRange& range) : m_Step(step)
    {
        for (float i = range.fmin(); i < range.fmax(); i += step) {
            m_Range.push_back(i);
        }
    }

    std::vector<float>::const_iterator begin() const
    {
        return m_Range.cbegin();
    }

    std::vector<float>::const_iterator end() const
    {
        return m_Range.cend();
    }

    float fmin() const
    {
        if (m_Range.empty()) {
            return 0.0;
        }
        return m_Range.front();
    }

    float fmax() const
    {
        if (m_Range.empty()) {
            return 0.0;
        }
        return m_Range.back();
    }

    std::size_t size() const
    {
        return m_Range.size();
    }

    std::size_t steps() const
    {
        return size();
    }

    float step() const
    {
        return m_Step;
    }

    float bandwidth() const
    {
        return m_Step;
    }

private:
    float m_Step;
    std::vector<float> m_Range;
};

uint32_t next_pow2(uint32_t x)
{
    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);

    return x + 1;
};

struct RadioHelper {
    RadioHelper() : radio(nullptr)
    {
    }

    RadioHelper(CaribouLiteRadio* a_radio) : radio(a_radio)
    {
        if (radio) {
            float n_sample_buffer_size = radio->GetNativeMtuSample();
            float samplerate = radio->GetRxSampleRateMax();

            unsigned scan_samples = next_pow2(std::floor(0.002 * samplerate));
            samples_per_buffer = scan_samples;

            sample_buffers_per_scan = std::ceil(scan_samples / n_sample_buffer_size);

            std::cout << "Sample buffer size : " << std::dec << n_sample_buffer_size << std::endl;
            std::cout << "Samples for 2ms    : " << std::dec << scan_samples << std::endl;
            std::cout << "Sample buffers per scan :" << std::dec << sample_buffers_per_scan << std::endl;

            frequencyranges = radio->GetFrequencyRange();
            for (auto& r : frequencyranges) {
                frequencies.emplace_back(ScanRange(radio->GetRxBandwidthMax(), r));
            }
        }
    }

    CaribouLiteRadio* radio;
    std::vector<CaribouLiteFreqRange> frequencyranges;
    std::vector<ScanRange> frequencies;

    unsigned sample_buffers_per_scan;
    std::size_t samples_per_buffer;
};

// Print Board Information
void printInfo(CaribouLite& cl)
{
    std::cout << "Initialized CaribouLite: " << cl.IsInitialized() << std::endl;
    std::cout << "API Versions: " << cl.GetApiVersion() << std::endl;
    std::cout << "Hardware Serial Number: " << std::hex << cl.GetHwSerialNumber() << std::endl;
    std::cout << "System Type: " << cl.GetSystemVersionStr() << std::endl;
    std::cout << "Hardware Unique ID: " << cl.GetHwGuid() << std::endl;
}

// Detect the board before instantiating it
void detectBoard()
{
    CaribouLite::SysVersion ver;
    std::string name;
    std::string guid;

    if (CaribouLite::DetectBoard(&ver, name, guid)) {
        std::cout << "Detected Version: " << CaribouLite::GetSystemVersionStr(ver) << ", Name: " << name << ", GUID: " << guid << std::endl;
    } else {
        std::cout << "Undetected CaribouLite!" << std::endl;
    }
}

class SampleBuffer
{
public:
    SampleBuffer(std::size_t scan_cycle, std::size_t size, const std::string radioName) : m_ScanCycle(scan_cycle), m_RadioName(radioName)
    {
        m_Data.resize(size);
        m_Timestamp = std::chrono::system_clock::now();
    };

    std::size_t cycle() const
    {
        return m_ScanCycle;
    }

    float freq() const
    {
        return m_Freq;
    }

    void setFreq(const float& new_freq)
    {
        m_Freq = new_freq;
    }

    std::vector<std::complex<int16_t>>& data()
    {
        return m_Data;
    }

    const std::vector<std::complex<int16_t>>& data() const
    {
        return m_Data;
    }

    const std::string& radioname() const
    {
        return m_RadioName;
    }

    const std::chrono::system_clock::time_point& timestamp() const
    {
        return m_Timestamp;
    }

private:
    std::vector<std::complex<int16_t>> m_Data;
    float m_Freq{0};
    std::chrono::system_clock::time_point m_Timestamp;
    const std::size_t m_ScanCycle;
    const std::string m_RadioName;
};

std::string format_freq(float frequency)
{
    std::array<char, 128> buf;

    if (frequency > 1e6) {
        std::snprintf(buf.data(), buf.size(), "%.3f MHz", (frequency / 1e6));
    } else if (frequency > 1e3) {
        std::snprintf(buf.data(), buf.size(), "%.3f KHz", (frequency / 1e3));
    } else {
        std::snprintf(buf.data(), buf.size(), "%.0f Hz", frequency);
    }

    return std::string(buf.data());
}

int write_all(int fd, const uint8_t* ptr, std::size_t size)
{
    std::size_t nwritten = 0;
    while (nwritten < size) {
        int ret = write(fd, ptr + nwritten, size - nwritten);
        if (ret == -1) {
            if (errno != EAGAIN) {
                return errno;
            }
        } else {
            nwritten += ret;
        }
    }
    return 0;
}

template <typename T> int write_all(int fd, const std::vector<T>& data)
{
    std::size_t size = data.size() * sizeof(data[0]);
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(data.data());
    return write_all(fd, ptr, size);
}

int write_all(int fd, const std::string& data)
{
    return write_all(fd, reinterpret_cast<const uint8_t*>(data.c_str()), data.size());
}

std::string timeToUtcString(const std::chrono::system_clock::time_point& tp)
{
    auto tnow = std::chrono::system_clock::to_time_t(tp);
    auto tfrac = std::chrono::duration_cast<std::chrono::milliseconds>(tp - std::chrono::system_clock::from_time_t(tnow));

    std::string ret;

    std::array<char, 128> buf;
    strftime(buf.data(), buf.size(), "%Y-%m-%dT%H:%M:%S.%%03uZ", gmtime(&tnow));
    std::array<char, 128> buf2;
    std::snprintf(buf2.data(), buf2.size(), buf.data(), (unsigned)tfrac.count());

    return std::string(buf2.data());
}

std::filesystem::path getUserHomeDir()
{
    return std::filesystem::path(getpwuid(getuid())->pw_dir);
}

class SampleStore
{
public:
    SampleStore(std::size_t cycle) : m_Cycle(cycle)
    {
        std::array<char, 256> filename;
        auto tnow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        std::snprintf(filename.data(), filename.size(), "samples-%09zu-%09zu.sigmf-data", (std::size_t)tnow, cycle);
        m_DataFilename = getUserHomeDir() / "dumps" / std::string(filename.data());

        std::snprintf(filename.data(), filename.size(), "samples-%09zu-%09zu.sigmf-meta", (std::size_t)tnow, cycle);
        m_MetaFilename = getUserHomeDir() / "dumps" / std::string(filename.data());

        data_fd = open(m_DataFilename.c_str(), O_CREAT | O_WRONLY, 0644);
        if (data_fd == -1) {
            std::cerr << "Opening file " << m_DataFilename << " failed" << std::endl;
            std::exit(-1);
        }

        m_SampleMap.clear();
        m_AnnotationMap.clear();
        m_nSamples = 0;
    }

    void finalize()
    {
        if (data_fd != -1) {
            close(data_fd);
            data_fd = -1;

            /* Build a sigmf structure */
            nlohmann::json sigmf = nlohmann::json::object();
            {
                nlohmann::json global = nlohmann::json::object();
                global["core:datatype"] = "ci16_le";
                global["core:sample_rate"] = (uint64_t)4e6;
                global["core:hw"] = "CaribouLite-6G-01";
                global["core:author"] = "daemontech";
                global["core:version"] = "1.2.0";
                sigmf["global"] = global;
            }

            sigmf["captures"] = m_SampleMap;
            sigmf["annotations"] = m_AnnotationMap;

            auto sigmf_dump = sigmf.dump();

            {
                int meta_fd = open(m_MetaFilename.c_str(), O_CREAT | O_WRONLY, 0644);
                if (meta_fd == -1) {
                    std::cerr << "Opening file " << m_MetaFilename << " failed" << std::endl;
                    std::exit(-1);
                }
                int ret = write_all(meta_fd, sigmf_dump);
                close(meta_fd);
            }
        }
    }

    void append(std::shared_ptr<SampleBuffer> buffer)
    {
        {
            nlohmann::json obj;
            obj["core:sample_start"] = m_nSamples;
            obj["core:frequency"] = (uint64_t)buffer->freq();
            obj["core:datetime"] = timeToUtcString(buffer->timestamp());

            m_SampleMap.push_back(obj);
        }
        {
            nlohmann::json obj;
            obj["core:sample_start"] = m_nSamples;
            obj["core:sample_count"] = buffer->data().size();
            obj["core:freq_lower_edge"] = (int64_t)(buffer->freq() - 2e6);
            obj["core:freq_upper_edge"] = (int64_t)(buffer->freq() + 2e6);
            obj["core:label"] = format_freq(buffer->freq());
            m_AnnotationMap.push_back(obj);
        }

        int err = write_all(data_fd, buffer->data());
        if (err != 0) {
            std::cerr << "FATAL: Writing buffer file failed (" << strerror(err) << ")" << std::endl;
            std::exit(-1);
        }
        m_nSamples += buffer->data().size();
    }

    ~SampleStore()
    {
        finalize();
    }

    std::size_t total_size() const
    {
        return m_nSamples;
    }

    std::size_t cycle() const
    {
        return m_Cycle;
    }

private:
    std::filesystem::path m_MetaFilename;
    std::filesystem::path m_DataFilename;

    std::vector<nlohmann::json> m_SampleMap;
    std::vector<nlohmann::json> m_AnnotationMap;

    int data_fd{-1};

    std::size_t m_nSamples{0};
    std::size_t m_Cycle;
};

class SampleProcessor
{
public:
    SampleProcessor() : m_KeepRunning(true)
    {
        m_Worker = std::thread([this]() { outerloop(); });
    };

    ~SampleProcessor()
    {
        m_KeepRunning = false;
        m_cvBuffer.notify_all();
        m_Worker.join();
    }

    void submit(std::shared_ptr<SampleBuffer> buffer)
    {
        {
            std::unique_lock<std::mutex> lock(m_mtxBuffer);
            m_Buffers.push_back(buffer);
        }
        m_cvBuffer.notify_all();
    }

    bool ready()
    {
        std::unique_lock<std::mutex> lock(m_mtxBuffer);
        return m_Buffers.empty();
    }

private:
    void outerloop()
    {
        std::unique_lock<std::mutex> lock(m_mtxBuffer, std::defer_lock_t());

        while (m_KeepRunning) {
            std::shared_ptr<SampleBuffer> buf;

            lock.lock();
            {
                if (!m_Buffers.empty()) {
                    buf = m_Buffers.front();
                    m_Buffers.pop_front();
                } else {
                    m_cvBuffer.wait_for(lock, std::chrono::milliseconds(200), [this] { return !m_KeepRunning || !m_Buffers.empty(); });
                }
            }
            lock.unlock();

            if (buf) {
                handleBuffer(buf);
            }
        }

        if (store) {
            store->finalize();
            store.reset();
        }
    }

    void handleBuffer(std::shared_ptr<SampleBuffer> buffer)
    {
        if (store && store->cycle() != buffer->cycle()) {
            store->finalize();
            store.reset();
        }

        if (!store) {
            store = std::make_unique<SampleStore>(buffer->cycle());
        }

        store->append(buffer);
    }

    std::deque<std::shared_ptr<SampleBuffer>> m_Buffers;
    std::mutex m_mtxBuffer;
    std::condition_variable m_cvBuffer;
    bool m_KeepRunning;

    std::thread m_Worker;
    std::unique_ptr<SampleStore> store;
};

// Main entry
int main()
{
    // try detecting the board before getting the instance
    detectBoard();

    // get driver instance - use "CaribouLite&" rather than "CaribouLite" (ref)
    // get a "synchronous" api instance
    auto& cl = CaribouLite::GetInstance(false);

    // print the info after connecting
    printInfo(cl);

    // get the radios
    std::vector<RadioHelper> radios;
    // radios.emplace_back(RadioHelper(cl.GetRadioChannel(CaribouLiteRadio::RadioType::S1G)));
    radios.emplace_back(RadioHelper(cl.GetRadioChannel(CaribouLiteRadio::RadioType::HiF)));

    for (auto& r : radios) {
        std::cout << "Radio :" << r.radio->GetRadioName() << "  MtuSize : " << std::dec << r.radio->GetNativeMtuSample() << " Samples" << std::endl;
        std::cout << "  " << std::dec << r.sample_buffers_per_scan << " sample buffers per scan" << std::endl;
        std::cout << "  Frequency Regions : " << std::endl;
        unsigned n = 0;
        for (auto i = r.frequencies.begin(); i < r.frequencies.end(); i = std::next(i)) {
            std::cout << " - " << n << " : " << i->fmin() << " - " << i->fmax() << " : " << i->steps() << " steps " << std::endl;
        }
    }

    SampleProcessor processor;

    std::size_t led_blink = 0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        led_blink++;

        cl.SetLed0States(led_blink & 16);
        if (cl.GetButtonState()) {
            break;
        }
    }
    cl.SetLed0States(false);

    std::size_t cycle = 1;
    while (true) {
        std::size_t n_samples_total = 0;
        for (auto& r : radios) {
            for (auto& band : r.frequencies) {
                try {
                    r.radio->SetFrequency(band.fmin());
                    r.radio->SetRxGain(69);
                    r.radio->SetAgc(false);
                    r.radio->SetRxSampleRate(4000000);
                    r.radio->SetRxBandwidth(band.bandwidth());
                    r.radio->FlushBuffers();
                    r.radio->StartReceiving();

                    for (auto freq = band.begin(); freq < band.end(); ++freq) {
                        // Receive Synchrnonously
                        try {
                            r.radio->SetFrequency(*freq, false);
                            r.radio->FlushBuffers();

                            std::size_t n_buffer_size = std::min(r.samples_per_buffer, r.radio->GetNativeMtuSample());

                            int num_buffers = r.sample_buffers_per_scan;
                            while (num_buffers--) {
                                auto buf = std::make_shared<SampleBuffer>(cycle, n_buffer_size, r.radio->GetRadioName());

                                int ret = r.radio->ReadSamples(buf->data().data(), n_buffer_size);
                                if (ret > 0) {
                                    buf->data().resize(std::min((std::size_t)ret, n_buffer_size));
                                    buf->setFreq(r.radio->GetFrequency());
                                    n_samples_total += buf->data().size();
                                    processor.submit(buf);
                                }
                            }
                        } catch (...) {
                            std::cout << "Radio " << r.radio->GetRadioName() << ", Capturing at " << format_freq(*freq) << " failed " << std::endl;
                            std::cout << " -- The specified freq couldn't be used" << std::endl;
                        }
                    }

                } catch (...) {
                    std::cout << "Radio " << r.radio->GetRadioName() << ", Capturing at " << format_freq(band.fmin()) << " failed " << std::endl;
                    std::cout << " -- The specified freq couldn't be used" << std::endl;
                }
            }
        }
        std::cout << "Ready, received " << std::dec << n_samples_total << " samples for 1 scan" << std::endl;
        ++cycle;

        while (!processor.ready()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
    }

    return 0;
}
