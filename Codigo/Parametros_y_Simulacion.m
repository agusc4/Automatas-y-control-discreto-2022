clc; clear; close all;
%% Par�metros del Carro

%Par�metros dato
m_c = 50000.0;                  % Masa total del carro [kg]
J_m = 10.0;                     % Momento de inercia del motor [kg * m^2]
J_r = 2.0;                      % Momento de inercia de la rueda [kg * m^2]
R_r = 0.5;                      % Radio primitivo de la rueda [m]
r_t = 15.0;                     % Relaci�n de transmisi�n
b_eq = 30.0;                    % Amortiguamiento viscoso del lado motor [N * m / (rad/s)]
y_t0 = 45;                      % Altura del riel del carro [m]

x_t0 = -20;                     % Posici�n inicial del carro [m]
v_c_max = 4;                    % Velocidad m�xima que puede alcanzar el carro [m/s]
a_c_max = 1;                    % Aceleraci�n m�xima que puede alcanzar el carro [m/s^2]

% Par�metros calculados
m_eqc = m_c + (J_r + J_m * r_t^2) / (R_r^2); % Masa equivalente [kg]
b_eqc = b_eq * r_t^2 / R_r^2;                % Amortiguamiento viscoso equivalente [N * m / (rad/s)]

%% Par�metros del Sistema de Izaje

%Par�metros dato
J_d = 8.0;                      % Momento de inercia del tambor [kg * m^2]
J_mi = 30.0;                    % Momento de inercia del motor + freno [kg * m^2]
R_d = 0.75;                     % Radio primitivo de tambor [m]
r_ti = 30.0;                    % Relaci�n de transmisi�n
b_eq2 = 18.0;                   % Amortiguamiento viscoso del lado motor [N * m / (rad/s)] 

v_i_max_c = 1.5;                % Velocidad m�xima que puede alcanzar el tambor cargado [m/s]
v_i_max_v = 3;                  % Velocidad m�xima que puede alcanzar el tambor descargado [m/s]
a_i_max = 1;                    % Aceleraci�n m�xima que puede alcanzar el tambor [m/s^2]

% Par�metros equivalentes izaje
m_eqi = (J_d + J_mi * r_ti^2) / R_d^2;       % Masa equivalente [kg]
b_eqi = b_eq2 * r_ti^2 / R_d^2;              % Amortiguamiento viscoso equivalente [N * m / (rad/s)]

%% Par�metros relacionados con la carga

% Par�metros del cable
K_w = 1800000.0;                % Rigidez a la tracci�n [N / m]
b_w = 30000.0;                  % Amortiguamiento viscoso propio [N / (m/s)]
K_cy = 1.3e6;                   % Rigidez y friccion vertical [N / m]
b_cy = 500.0;                   % Amortiguamiento viscoso vertical [N / (m/s)]
b_cx = 1000.0;                  % Amortiguamiento viscoso horizontal [N / (m/s)]

% Par�metros del contenedor
m_spreader = 15000.0;                        % Masa del gancho [kg]
m_contenedor = 2000.0;                       % Masa del contenedor vac�o [kg]
H_contenedor = 2.5;                          % Altura del contenedor [m]
L_contenedor = 2.5;                          % Ancho del contenedor [m]
H_seguridad = 5;                             % Altura de seguridad para evitar colisiones [m] // VARIAR A GUSTO

m_carga = 48000.0;                           % Masa de la carga [kg] // VARIAR A GUSTO
m_carga_min = m_spreader + m_contenedor;     % Masa m�nima transportada en carga [kg]
m_carga_max = m_carga_min + m_carga;         % Masa m�xima transportada en carga [kg]
g = 9.81;                                    % Aceleraci�n de la gravedad [m/s^2]

l_t0 = y_t0 - H_contenedor - H_seguridad -(m_spreader * g) / K_w;    % Altura inicial de la carga [m]

%% Par�metros de Sensores y Controladores

hLimSup = 40+2;                     % Ubicaci�n del actuador para el fin de carrera superior [m]
hLimInf = -17.5-2;                  % Ubicaci�n del actuador para el fin de carrera inferior [m]
tLimIzq = -30-2;                    % Ubicaci�n del actuador para el fin de carrera izquierdo [m]
tLimDer = 50+2;                     % Ubicaci�n del actuador para el fin de carrera derecho [m]

omega_c_mod = 1000;                 % Polo designado para el modulador de torque del carro
omega_i_mod = 1000;                 % Polo designado para el modulador de torque del sistema de izaje
tau = 1/1000;
%% Par�metros Nivel 2

[B_ac, K_sac, K_siac] = PID_carro(b_eqc, m_eqc);
[B_ai, K_sai, K_siai] = PID_izaje(b_eqi, m_eqi);
[lh,K_db,K_pb] = PD_balanceo(m_c, m_carga_max, m_eqc);

%% Gr�ficos

L_colmn = [5,10,15, 20,25, 30,35, 40,45];
H_colmn = [34,34, 19,19, 35,30, 35,30, 23];
[L, H] = plot_estatico(L_colmn, H_colmn);

%% Simulaci�n

% Sistema_completo para simulaci�n local. Sistema_completo_server para
% simulaci�n integrada con Codesys.
