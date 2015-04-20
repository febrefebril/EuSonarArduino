// #####################################################################################
// #                         Projeto EuSonar - Modulo Bone                             #
// #                         Autor: Andre Perotti Netto                                #
// #                         Data criacao: 04/11/2015                                  #
// #####################################################################################
// Prototipo do bone que visa evitar obstaculos

#include<NewPing.h>
// Constantes de configuracao geral
#define DelayEntrePing              75  // Tempo em ms de espera entre as leituras ultrasom
#define DistanciaMediaBraco         50  // Distancia media braco humano
#define DistanciaMediaPasso         90  // Distancia media passo humano
#define Desligado                    0  
#define DistanciaMaximaUltrasonico 300  // Distancia maxima de leitura do ultrasom 3 m
#define ForcaMediaMotor             40  // TODO: medir a forca media e alterar
#define ForcaAltaMotor              80  // TODO: medir forca maxima do motor e alterar
// Constantes que definem o inicio, fim de comando
#define InicioDeComando            '*'
#define FimDeComando               '#'
// Constantes que representam os motores de vibracao
#define MotorFrontal                11  // Pinos que estao ligados no motor frontal
#define MotorEsquerdo               10  // Pinos que estao ligados no motor esquerdo
#define MotorDireito                 9  // Pinos que estao ligados no motor direito
#define MotorTrazeiro                6  // Pinos que estao ligados no motor trazeiro

// Constantes do sensor ultrasonico frontal
#define UltrasonicoFrontalTrigger   12  // Numero do pino em que sera ligado o trigger
#define UltrasonicoFrontalEcho       8  // Numero do pino em que sera legado o echo

// Constantes do sensor ultrasonico da lateral esquerda
#define UltrasonicoEsquerdoTrigger   7  // Numero do pino em que sera ligado o trigger
#define UltrasonicoEsquerdoEcho      5  // Numero do pino em que sera legado o echo

// Constantes do sensor ultrasonico da lateral direita
#define UltrasonicoDireitoTrigger    4  // Numero do pino em que sera ligado o trigger
#define UltrasonicoDireitoEcho       3  // Numero do pino em que sera legado o echo

#define QtdSonares                   3  // Quantidade de senssores ultra som
#define IntervaloEntrePings         33  // Tempo em ms entre pings evitar echo cruzados 

// Estrutura que controla em qual estado do programa estamos rodando
enum EstadoDeExecucao
{
    MotoresDesligados=1,  // Motores desligados.
    NavegacaoComMotores
};

typedef enum EstadoDeExecucao estado;

// inicia objetos que implemetam o sensores ultrasonico
NewPing UltrasomFrontal(
    UltrasonicoFrontalTrigger,
    UltrasonicoFrontalEcho,
    DistanciaMaximaUltrasonico
);

NewPing UltrasomLateralEsquerdo(
    UltrasonicoEsquerdoTrigger,
    UltrasonicoEsquerdoEcho,
    DistanciaMaximaUltrasonico
);

NewPing UltrasomLateralDireito(
    UltrasonicoDireitoTrigger,
    UltrasonicoDireitoEcho,
    DistanciaMaximaUltrasonico
);

//  Valores que conterao o tempo de espera de cada sensor
//  ultrasom antes de uma nova leitura
unsigned long CronometroPing[QtdSonares];
unsigned int UltraSomAtual = 0;  // Mantem qual ultrasom esta ativo

//  somente valores positivos e por isso serao unsigned
unsigned int DistanciaFrontal;
unsigned int DistanciaFrontalAnterior;
unsigned int DistanciaLateralEsquerda;
unsigned int DistanciaLateralEsquerdaAnterior;
unsigned int DistanciaLateralDireita;
unsigned int DistanciaLateralDireitaAnterior;
unsigned int Duracao;
unsigned int Tempo;
//  declara o estado inicial que representa o que o app
//  esta fazendo no momento
estado estado_atual_app;
// TODO: mudar esta string que serve para pegar parametro
String string;
char o_comando;

void setup()
{

    // Inicializa serial que fara comunicacao com andoid
    Serial.begin(9600);
    Serial.println("Ola, EuSonar");
    Serial.flush();
    CronometroPing[0] = millis() + 75;  // Primeiro ping começa as 75ms
    for(unsigned int i = 1; i < QtdSonares; i++)
    {
        CronometroPing[i] = CronometroPing[i-1] + IntervaloEntrePings;
    }

    // Inicializa as variaveis que sao sobrescritas apos primeira leitura
    DistanciaFrontal = 0;
    DistanciaFrontalAnterior = 0;
    DistanciaLateralEsquerda = 0;
    DistanciaLateralEsquerdaAnterior = 0;
    DistanciaLateralDireita = 0;
    DistanciaLateralDireitaAnterior = 0;

    estado_atual_app = NavegacaoComMotores;

    // Inicia pinos dos motores como saida
    pinMode(MotorFrontal, OUTPUT);
    pinMode(MotorEsquerdo, OUTPUT);
    pinMode(MotorDireito, OUTPUT);
    pinMode(MotorTrazeiro, OUTPUT);
}

void loop()
{

    Serial.flush();
    if ( Serial.available() < 1 )
    {
        acao_padrao_quando_nenhuma_leitura_foi_feita();  // NavegacaoComMotores
    }
    else
    {
        if( Serial.available() > 0 )
        {
            string = "";
        }
        while( Serial.available() > 0 )
        {
            o_comando = ( (byte)Serial.read() );
            if( o_comando == ':' )
            {
                break;
            }
            else
            {
                string += o_comando; 
            }
            delay(1);  // Por motivos puramente exotericos e numerologicos
        }
    }

    trata_comando_lido_pelo_bluetooth();
    // Independentemente do estado, sempre leio o ultrasom.
    scan();
    // Leio bluetooth para pegar o estado de execucao
    // sua respectiva funcao
    switch( estado_atual_app )
    {
        case MotoresDesligados :
            estado_motores_desligados();
            break;
        case NavegacaoComMotores :
            estado_navegacao_com_motores();

        default:
            estado_navegacao_com_motores();
            Serial.println("Default atingido");
    }
    
}

// Faz a leitura de todos os sensores ultrasom e seta as variaveis com as distancias lidas.
void scan()
{// TODO: Refatorar esta funcao!!!
    Serial.println("Entrando em scan");
    if(millis() >= CronometroPing[0])
    {
        // Esta na hora deste senssor pingar?
        Serial.println("Tratando Sensor UltrasomFrontal.");
        Serial.print("millis(): ");
        Serial.print(millis());
        Serial.print(" | ");
        Serial.print("CronometroPing sensor frontal: ");
        Serial.print(CronometroPing[0]);
        Serial.println("");
        CronometroPing[0] += IntervaloEntrePings * QtdSonares;
        Serial.print("Novo valor de CronometroPing sensor frontal:");
        Serial.print(CronometroPing[0]);
        Serial.println("");
        DistanciaFrontalAnterior = DistanciaFrontal;
        DistanciaFrontal = Tempo = 0;// Limpa as duas variaveis antes de usar
        //delay(DelayEntrePing);
        //UltrasomFrontal.timer_stop();
        Tempo = UltrasomFrontal.ping_median(5);
        DistanciaFrontal = Tempo / US_ROUNDTRIP_CM;
        Serial.print("Distancia Frontal: ");
        Serial.print(DistanciaFrontal);
        Serial.println("");
    }

    if(millis() >= CronometroPing[1])
    {
        // Esta na hora deste senssor pingar?
        Serial.println("Tratando Sensor UltrasomLateralEsquerdo.");
        Serial.print("millis(): ");
        Serial.print(millis());
        Serial.print(" | ");
        Serial.print("CronometroPing sensor lateral esquerdo: ");
        Serial.print(CronometroPing[1]);
        Serial.println("");
        CronometroPing[1] += IntervaloEntrePings * QtdSonares;
        Serial.print("Novo valor de CronometroPing sensor lateral esquerdo:");
        Serial.print(CronometroPing[1]);
        Serial.println("");
        DistanciaLateralEsquerdaAnterior = DistanciaLateralEsquerda;
        DistanciaLateralEsquerda = Tempo = 0;// Limpa as duas variaveis antes de usar
        //delay(DelayEntrePing);
        //UltrasomLateralEsquerdo.timer_stop();
        Tempo = UltrasomLateralEsquerdo.ping_median(5);
        DistanciaLateralEsquerda = Tempo / US_ROUNDTRIP_CM;
        Serial.print("Distancia Lateral Esquerda: ");
        Serial.print(DistanciaLateralEsquerda);
        Serial.println("");
    }

    if(millis() >= CronometroPing[2])
    {
        // Esta na hora deste senssor pingar?
        Serial.println("Tratando Sensor UltrasomLateralDireito.");
        Serial.print("millis(): ");
        Serial.print(millis());
        Serial.print(" | ");
        Serial.print("CronometroPing sensor lateral direito: ");
        Serial.print(CronometroPing[2]);
        Serial.println("");
        CronometroPing[2] += IntervaloEntrePings * QtdSonares;
        Serial.print("Novo valor de CronometroPing lateral direito:");
        Serial.print(CronometroPing[2]);
        Serial.println("");
        DistanciaLateralDireitaAnterior = DistanciaLateralDireita;
        DistanciaLateralDireita = Tempo = 0;// Limpa as duas variaveis antes de usar
        //delay(DelayEntrePing);
        //UltrasomLateralDireito.timer_stop();
        Tempo = UltrasomLateralDireito.ping_median(5);
        DistanciaLateralDireita = Tempo / US_ROUNDTRIP_CM;
        Serial.print("Distancia Lateral Direito: ");
        Serial.print(DistanciaLateralDireita);
        Serial.println("");
    }
}

// De acordo com as distancias lidas por scan() vibra os motores.
void navegacao_com_motor()
{
    // TODO: chamar as funcoes que tratam as distancias e vibrem os motores
    tratamento_motor(MotorFrontal);
    tratamento_motor(MotorEsquerdo);
    tratamento_motor(MotorDireito);
}

void tratamento_motor(int posicao)
{
    Serial.println("Entrando em tratamento_motor()");
    int distancia_medida = 0; // vai receber a distancia de um dos sensores
    // Verifa se a distancia aumentou ou diminuio.
    int comparacao_distancia = compara_distancias(DistanciaFrontal, DistanciaFrontalAnterior);
    if( posicao ==  MotorFrontal)
    {
        Serial.println("Trantando motor Frontal");
        distancia_medida = DistanciaFrontal;
    }
    else if ( posicao == MotorEsquerdo )
    {
        Serial.println("Trantando motor Lateral esquerdo");
        distancia_medida = DistanciaLateralEsquerda;
    }
    else if ( posicao == MotorDireito )
    {
        Serial.println("Trantando motor Lateral Direito");
        distancia_medida = DistanciaLateralDireita;
    }
    if( distancia_medida > DistanciaMediaPasso || distancia_medida == Desligado )
    {
        // Distancia segura, desliga motores de vibracao.
        // TODO: fazer funcao que desligue os motores.
        Serial.print("Desligando Motor: ");
        Serial.print(posicao);
        Serial.println("");
        desliga_motor(posicao);
    }
    // Caso exista um objeto entre 50 e 90 cm de um aviso vibratorio medio.
    else if ( (distancia_medida > DistanciaMediaBraco || distancia_medida == Desligado) &&
              (distancia_medida < DistanciaMediaPasso || distancia_medida == Desligado) )
    {
        // TODO: chamar funcao que vibra com potencia media
        // Compara os valores para saber se o objeto esta se aproximando e da um alerta medio
        Serial.print("Objeto proximo encontrado na posicao: ");
        Serial.print(posicao);
        Serial.println("");
        trata_retorno_das_distancia_com_vibracao(posicao, comparacao_distancia, ForcaMediaMotor);
    }
    // Caso em que o objeto se encontra em uma distancia menor que 50 cm, alerta com vibracao forte.
    else
    {
        // TODO: chamar funcao que vibra com potencia forte
        Serial.print("ATENCAO: Objeto muito proximo ao sensor: ");
        Serial.print(posicao);
        Serial.println("");
        trata_retorno_das_distancia_com_vibracao(posicao, comparacao_distancia, ForcaAltaMotor);
    } 
}

// Retorna a diferenca do valor atual com valor anterior
int compara_distancias(unsigned int valor_atual, unsigned int valor_anterior)
{
    return valor_atual - valor_anterior;
}
// Se a distancia esta diminuindo continua vibrando se a distancia se estabeliza ou aumenta, para.
void trata_retorno_das_distancia_com_vibracao(int posicao, int diferenca_das_distancias, int intencidade)
{
        if( diferenca_das_distancias < 0 )
        {
            // Distancia esta diminuido, aumenta vibracao ou continua o aviso
            // TODO: fazer funcao que aumenta vibracao
            if( intencidade == ForcaMediaMotor)
            {
                //TODO: Fazer funcao que liga o motor com intensidade media.
                liga_motor(posicao, ForcaMediaMotor); // Fazer esta funcao.
            }
            else if ( intencidade == ForcaAltaMotor)
            {
                //TODO: Fazer funcao que liga o motor com intensidade forte.
                liga_motor(posicao, ForcaAltaMotor); // Fazer esta funcao.
            }
        }
        else
        {
            // Distancia se manteve ou esta aumento, diminui vibracao ou pare de vibrar
            // TODO: Fazer funcao que diminui vibracao ou deliga motor, por enquanto desligamos.
            desliga_motor(posicao); // Fazer esta funcao.
        }
}
// Desliga Motor do pino passado como parametro posicao_motoresligar

void desliga_motor(int posicao_motor_desligar)
{
    analogWrite(posicao_motor_desligar, 0);
}
// Liga motor 
void liga_motor(int posicao_motor_alterar, int forca)
{
    analogWrite(posicao_motor_alterar, forca);

}
// Acao tomada caso nao esiver recebendo nada pelo bluetooth
void acao_padrao_quando_nenhuma_leitura_foi_feita()
{
    estado_atual_app = NavegacaoComMotores;
    Serial.println("Acao padrao quando nenhuma leitura por Bluetooth feita.");
    Serial.print("Estado Atual: ");
    Serial.print(estado_atual_app);
    Serial.println("");

}
// Pega valor lido e pelo bluetooth e seta estado do app pelo seu valor correspondente
// Quando possivel alterar esta grande arvore de natal :D
void trata_comando_lido_pelo_bluetooth()
{
    Serial.println("Tratando comando lido pelo Bluetooth.");

    if(string == "MotoresDesligados")
    {
        Serial.println("Estado: MotoresDesligados");
        estado_atual_app = MotoresDesligados;
        estado_motores_desligados();
    }
    else
    {  // Valor padrao.
        Serial.println("Estado: NavegacaoComMotores");
        estado_atual_app = NavegacaoComMotores;
    }
    Serial.print("Estado atual: ");
    Serial.print(estado_atual_app);
    Serial.println("");
}

void estado_motores_desligados()
{
    Serial.println("estado_motores_desligados");
    desliga_motor(MotorFrontal);
    desliga_motor(MotorEsquerdo);
    desliga_motor(MotorDireito);
    desliga_motor(MotorTrazeiro);
}
void estado_navegacao_com_motores()
{
    Serial.println("estado_navegacao_com_motore");
    navegacao_com_motor();
}
