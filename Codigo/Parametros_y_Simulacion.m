clc; clear; close all;
%% Parámetros del Carro

%Parámetros dato
m_c = 50000.0;                  % Masa total del carro [kg]
J_m = 10.0;                     % Momento de inercia del motor [kg * m^2]
J_r = 2.0;                      % Momento de inercia de la rueda [kg * m^2]
R_r = 0.5;                      % Radio primitivo de la rueda [m]
r_t = 15.0;                     % Relación de transmisión
b_eq = 30.0;                    % Amortiguamiento viscoso del lado motor [N * m / (rad/s)]
y_t0 = 45;                      % Altura del riel del carro [m]

x_t0 = -20;                     % Posición inicial del carro [m]
v_c_max = 4;                    % Velocidad máxima que puede alcanzar el carro [m/s]
a_c_max = 1;                    % Aceleración máxima que puede alcanzar el carro [m/s^2]

% Parámetros calculados
m_eqc = m_c + (J_r + J_m * r_t^2) / (R_r^2); % Masa equivalente [kg]
b_eqc = b_eq * r_t^2 / R_r^2;                % Amortiguamiento viscoso equivalente [N * m / (rad/s)]

%% Parámetros del Sistema de Izaje

%Parámetros dato
J_d = 8.0;                      % Momento de inercia del tambor [kg * m^2]
J_mi = 30.0;                    % Momento de inercia del motor + freno [kg * m^2]
R_d = 0.75;                     % Radio primitivo de tambor [m]
r_ti = 30.0;                    % Relación de transmisión
b_eq2 = 18.0;                   % Amortiguamiento viscoso del lado motor [N * m / (rad/s)] 

v_i_max_c = 1.5;                % Velocidad máxima que puede alcanzar el tambor cargado [m/s]
v_i_max_v = 3;                  % Velocidad máxima que puede alcanzar el tambor descargado [m/s]
a_i_max = 1;                    % Aceleración máxima que puede alcanzar el tambor [m/s^2]

% Parámetros equivalentes izaje
m_eqi = (J_d + J_mi * r_ti^2) / R_d^2;       % Masa equivalente [kg]
b_eqi = b_eq2 * r_ti^2 / R_d^2;              % Amortiguamiento viscoso equivalente [N * m / (rad/s)]

%% Parámetros relacionados con la carga

% Parámetros del cable
K_w = 1800000.0;                % Rigidez a la tracción [N / m]
b_w = 30000.0;                  % Amortiguamiento viscoso propio [N / (m/s)]
K_cy = 1.3e6;                   % Rigidez y friccion vertical [N / m]
b_cy = 500.0;                   % Amortiguamiento viscoso vertical [N / (m/s)]
b_cx = 1000.0;                  % Amortiguamiento viscoso horizontal [N / (m/s)]

% Parámetros del contenedor
m_spreader = 15000.0;                        % Masa del gancho [kg]
m_contenedor = 2000.0;                       % Masa del contenedor vacío [kg]
H_contenedor = 2.5;                          % Altura del contenedor [m]
L_contenedor = 2.5;                          % Ancho del contenedor [m]
H_seguridad = 5;                             % Altura de seguridad para evitar colisiones [m] // VARIAR A GUSTO

m_carga = 48000.0;                           % Masa de la carga [kg] // VARIAR A GUSTO
m_carga_min = m_spreader + m_contenedor;     % Masa mínima transportada en carga [kg]
m_carga_max = m_carga_min + m_carga;         % Masa máxima transportada en carga [kg]
g = 9.81;                                    % Aceleración de la gravedad [m/s^2]

l_t0 = y_t0 - H_contenedor - H_seguridad -(m_spreader * g) / K_w;    % Altura inicial de la carga [m]

%% Parámetros de Sensores y Controladores

hLimSup = 40+2;                     % Ubicación del actuador para el fin de carrera superior [m]
hLimInf = -17.5-2;                  % Ubicación del actuador para el fin de carrera inferior [m]
tLimIzq = -30-2;                    % Ubicación del actuador para el fin de carrera izquierdo [m]
tLimDer = 50+2;                     % Ubicación del actuador para el fin de carrera derecho [m]

omega_c_mod = 1000;                 % Polo designado para el modulador de torque del carro
omega_i_mod = 1000;                 % Polo designado para el modulador de torque del sistema de izaje
tau = 1/1000;
%% Parámetros Nivel 2

[B_ac, K_sac, K_siac] = PID_carro(b_eqc, m_eqc);
[B_ai, K_sai, K_siai] = PID_izaje(b_eqi, m_eqi);
[lh,K_db,K_pb] = PD_balanceo(m_c, m_carga_max, m_eqc);

%% Gráficos

L_colmn = [5,10,15, 20,25, 30,35, 40,45];
H_colmn = [34,34, 19,19, 35,30, 35,30, 23];
[L, H] = plot_estatico(L_colmn, H_colmn);

%% Simulación

% Sistema_completo para simulación local. Sistema_completo_server para
% simulación integrada con Codesys.
