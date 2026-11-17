// Loads command grammars and parses strings only. No ControlSystem, prepareNrt,
// EtherCAT, robot subsystem, network connection, or execution is created here.
#include <codeit/core/command.hpp>
#include <codeit/function/motionfunc.hpp>
#include <codeit/function/planfunc.hpp>
#include <codeit/function/systemfunc.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>

int main(int argc,char** argv) {
  if(argc!=2)return 2;
  try {
    codeit::function::MoveAbsJ joint;
    codeit::function::MoveBlend line;
    codeit::function::Sleep sleep;
    codeit::core::CommandParser parser;
    parser.commandPool().add<codeit::core::Command>(joint.command());
    parser.commandPool().add<codeit::core::Command>(line.command());
    parser.commandPool().add<codeit::core::Command>(sleep.command());
    parser.commandPool().add<codeit::core::Command>().loadXmlStr(
      "<Command name=\"SetHrcsiDout\"><GroupParam><Param name=\"do_num\" default=\"-1\"/>"
      "<Param name=\"value\" default=\"-1\"/></GroupParam></Command>");
    parser.init();
    nlohmann::json frames;std::ifstream(argv[1])>>frames;
    int instructions=0;
    for(const auto& frame:frames) {
      std::string text=frame.at("command");
      if(text.front()!='{' || text.back()!='}')throw std::runtime_error("invalid multi-model frame");
      text=text.substr(1,text.size()-2);
      std::size_t begin=0;int models=0;
      while(true) {
        const auto end=text.find("||",begin);const auto command=text.substr(begin,end==std::string::npos?end:end-begin);
        ++models;
        if(command!="NotRunExecute") {parser.parse(command);++instructions;}
        if(end==std::string::npos)break;
        begin=end+2;
      }
      if(models!=3)throw std::runtime_error("wrong number of model slots");
    }
    std::cout<<"Codeit parser accepted "<<frames.size()<<" multi-model frames / "<<instructions<<" active subcommands; no execution performed\n";
    return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
