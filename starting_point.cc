//
// Created by Gastone Pietro Rosati Papini on 10/08/22.
//

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <agent_functions.hpp>

extern "C" {
#include "screen_print_c.h"
}
#include "screen_print.h"
#include "server_lib.h"
#include "logvars.h"

// --- MATLAB PRIMITIVES INCLUDE ---
// #include "primitives.h"
// --- MATLAB PRIMITIVES INCLUDE ---

#define DEFAULT_SERVER_IP    "127.0.0.1"
#define SERVER_PORT               30000  // Server port
#define DT 0.05

// Handler for CTRL-C
#include <signal.h>
static uint32_t server_run = 1;
void intHandler(int signal) {
    server_run = 0;
}

int main(int argc, const char * argv[]) {
    logger.enable(true);

    // Messages variables
    scenario_msg_t scenario_msg;
    manoeuvre_msg_t manoeuvre_msg;
    size_t scenario_msg_size = sizeof(scenario_msg.data_buffer);
    size_t manoeuvre_msg_size = sizeof(manoeuvre_msg.data_buffer);
    uint32_t message_id = 0;
    std::array<double,NUM_OF_COEFFS> m_star;
    std::string check= "Free";

#if not defined( _MSC_VER ) and not defined( _WIN32 )
    // More portable way of supporting signals on UNIX
    struct sigaction act;
    act.sa_handler = intHandler;
    sigaction(SIGINT, &act, NULL);
#else
    signal(SIGINT, intHandler);
#endif

    server_agent_init(DEFAULT_SERVER_IP, SERVER_PORT);

    // Start server of the Agent
    printLine();
    printTable("Waiting for scenario message...", 0);
    printLine(); 
    PIDController controller(0.045, 0.02);
    

    double prevTime = 0.0;
    std::string prev_primitive ="Free";
    while (server_run == 1) {

        // Clean the buffer
        memset(scenario_msg.data_buffer, '\0', scenario_msg_size);

        // Receive scenario message from the environment
        if (server_receive_from_client(&server_run, &message_id, &scenario_msg.data_struct) == 0) {
            // Init time
            static auto start = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::now()-start;
            double num_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(time).count()/1000.0;
            printLogTitle(message_id, "received message");
            double dt = num_seconds-prevTime;

            // Data struct
            input_data_str *in = &scenario_msg.data_struct;
            manoeuvre_msg.data_struct.CycleNumber = in->CycleNumber;
            manoeuvre_msg.data_struct.Status = in->Status;

            // Example of using log
            logger.log_var("Example", "cycle", in->CycleNumber);
            logger.log_var("Example", "vel", in->VLgtFild);

            // ADD AGENT CODE HERE
            double a0=in->ALgtFild;
            double v0=in->VLgtFild;
            double sf=in->TrfLightDist;
            double TH = 60 ;
            double lookhead = max(50, v0*5);
            double Vmin= 3;
            double Vmax=15;
            double Vr= in-> RequestedCruisingSpeed;
            double xs=10 ;
            double Ts = xs/Vmin;
            double xin=10;
            double Tin= xin/Vmin;
            double xtr= in->TrfLightDist;
            double T_green= 0,T_red=0;
            double xstop= xtr-(xs/2);
            double xf = sf - xs;
            
            
            if(in->NrTrfLights!=0){
                xtr= in->TrfLightDist;
                xstop= xtr-(xs/2);
                
            }
            if(in->NrTrfLights == 0 || xtr>= lookhead){
                m_star=choose_m_star(v0,a0,lookhead,Vr);
                logger.log_var("File", "primitives","Free");
                std::string current_primitive="Free";
                if(prev_primitive!=current_primitive){
                   controller.resetIntegral();
                   prev_primitive=current_primitive;
                }
            }
            else{
                switch(in->TrfLightCurrState){
                
                case 1:
                    T_green= 0;
                    T_red= in->TrfLightFirstTimeToChange-Tin;
                    break;
                case 2:
                    T_green=in->TrfLightSecondTimeToChange+Ts;
                    T_red=in->TrfLightThirdTimeToChange-Tin;
                    break;
                case 3:
                    T_green=in->TrfLightFirstTimeToChange+Ts;
                    T_red=in->TrfLightSecondTimeToChange-Tin;
                    break;
                }
                if(in->TrfLightCurrState==1 && in->TrfLightDist<=xs){
                m_star=choose_m_star(v0,a0,lookhead,Vr);
                logger.log_var("File", "primitives","Pass");
                std::string current_primitive="Pass";
                if(prev_primitive!=current_primitive){
                    controller.resetIntegral();
                    prev_primitive=current_primitive;
                }
                }
                else{
                    std::array<double,NUM_OF_COEFFS> m1, m2;
                    std::tie(m1, m2) = Passprimitive(a0,v0,xtr,Vmin,Vmax,T_green,T_red);
                    if(m1[2]==0 && m2[2]==0){
                        m_star=stopPrimitive(a0,v0,xstop);
                        logger.log_var("File", "xstop",xstop);
                        logger.log_var("File", "primitives","stop");
                        std::string current_primitive="stop";
                        if(prev_primitive!=current_primitive){
                           controller.resetIntegral();
                           prev_primitive=current_primitive;
                        }
                    }
                    else{
                        if((m1[2]<0 && m2[2]>0) || (m1[2]>0 && m2[2]<0)){
                            m_star=Passprimitve_with_j0(a0,v0,xtr,Vmin,Vmax);
                            logger.log_var("File", "primitives","pass with j0");
                            std::string current_primitive="pass with j0";
                            if(prev_primitive!=current_primitive){
                           	std::string current_primitive="Pass";
                            if(prev_primitive!=current_primitive){
                               controller.resetIntegral();
                               prev_primitive=current_primitive;
                            }
                            if (abs(m1[2])< abs(m2[2])){
                            m_star=m1;
                            
                            }
                            else{
                            m_star=m2;
                            }
                            
                        }
                    }
                }

            }  
            
            //logger.log_var("File","err",controller.eint);
                 

            // ADD LOW LEVEL CONTROL CODE HERE
            //manoeuvre_msg.data_struct.RequestedAcc = -0.3;
            manoeuvre_msg.data_struct.RequestedAcc = controller.getPedalRequest(a0, dt,m_star);
            
            manoeuvre_msg.data_struct.RequestedSteerWhlAg = 0.0;
            logger.log_var("File","final_a",manoeuvre_msg.data_struct.RequestedAcc);

            // Write log
            logger.write_line("File");
            logger.log_var("File", "sf",xf);
            logger.log_var("File", "v0",v0);
            logger.log_var("File", "Time",num_seconds);
            logger.log_var("File", "c1",m_star[0]);
            logger.log_var("File", "c2",m_star[1]);
            logger.log_var("File", "c3",m_star[2]);
            logger.log_var("File", "c4",m_star[3]);
            logger.log_var("File", "c5",m_star[4]);
            logger.log_var("File","a0",in->ALgtFild);

            // Screen print
            printLogVar(message_id, "Time", num_seconds);
            printLogVar(message_id, "Status", in->Status);
            printLogVar(message_id, "CycleNumber", in->CycleNumber);

            // Send manoeuvre message to the environment
            if (server_send_to_client(server_run, message_id, &manoeuvre_msg.data_struct) == -1) {
                perror("error send_message()");
                exit(EXIT_FAILURE);
            } else {
                printLogTitle(message_id, "sent message");
            }
        }
    }

    // Close the server of the agent
    server_agent_close();
    return 0;
}
