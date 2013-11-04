

class ComInterface
{
public:
   ComInterface{};
   virtual ~ComInterface{};

   virtual Read(const std::vector<std::uint8_t>& msg) = 0;
   virtual Write(std::vector<std::uint8_t>& msg) = 0;
};
