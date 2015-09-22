function varargout = fix_link_com_ui(varargin)
% FIX_LINK_COM_UI MATLAB code for fix_link_com_ui.fig
%      FIX_LINK_COM_UI, by itself, creates a new FIX_LINK_COM_UI or raises the existing
%      singleton*.
%
%      H = FIX_LINK_COM_UI returns the handle to a new FIX_LINK_COM_UI or the handle to
%      the existing singleton*.
%
%      FIX_LINK_COM_UI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FIX_LINK_COM_UI.M with the given input arguments.
%
%      FIX_LINK_COM_UI('Property','Value',...) creates a new FIX_LINK_COM_UI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before fix_link_com_ui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to fix_link_com_ui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help fix_link_com_ui

% Last Modified by GUIDE v2.5 20-Sep-2015 13:36:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @fix_link_com_ui_OpeningFcn, ...
                   'gui_OutputFcn',  @fix_link_com_ui_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT



% --- Executes just before fix_link_com_ui is made visible.
function fix_link_com_ui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to fix_link_com_ui (see VARARGIN)

% Choose default command line output for fix_link_com_ui
handles.output = hObject;

if ( nargin >= 4 )
    handles.drake_ik_interface = varargin{1};
else
    handles.drake_ik_interface = [];
end

% init ROS subscribers
handles = init_ros(handles);
handles = init_robot_model(handles);

% set selected link to utorso
tmp_cells = strfind({handles.robot_model.body.linkname}, 'utorso');
idx = find(~cellfun(@isempty,tmp_cells));

set(handles.current_link_combobox, 'String', {handles.robot_model.body.linkname});
set(handles.current_link_combobox, 'Value', idx);
current_link_combobox_Callback(handles.current_link_combobox, eventdata, handles);

update_robot_state_Callback(handles.update_robot_state, eventdata, handles);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes fix_link_com_ui wait for user response (see UIRESUME)
% uiwait(handles.fix_link_com_ui_fig);


% --- Outputs from this function are returned to the command line.
function varargout = fix_link_com_ui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in update_robot_state.
function update_robot_state_Callback(hObject, eventdata, handles)
% hObject    handle to update_robot_state (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    [joint_state_msg, ~, ~] = handles.joint_state_sub.poll(10);
    [imu_msg, ~, ~] = handles.imu_sub.poll(10);

    orientation_quat = [imu_msg.orientation.w;imu_msg.orientation.x;imu_msg.orientation.y;imu_msg.orientation.z];
    orientation_rpy = quat2rpy(orientation_quat);
    handles.current_robot_pose(4:6) = orientation_rpy;

    handles.current_robot_pose = handle_new_joint_state(joint_state_msg, handles.robot_model, handles.current_robot_pose);
    
    cla(handles.robot_com_figure);
    hull_distance = plot_robot_com(handles.robot_model, handles.current_robot_pose, 'CoM state');
    
    robot_com = handles.robot_model.getCOM(handles.current_robot_pose);
    set(handles.current_com_value, 'String', sprintf('% .3f   ', robot_com'));
    set(handles.hull_distance_value, 'String', sprintf('% .3f', hull_distance));
    guidata(hObject, handles);


% --- Executes on selection change in current_link_combobox.
function current_link_combobox_Callback(hObject, eventdata, handles)
% hObject    handle to current_link_combobox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns current_link_combobox contents as cell array
%        contents{get(hObject,'Value')} returns selected item from current_link_combobox

    link_names = get(handles.current_link_combobox, 'String');
    current_link_name = get(handles.current_link_combobox, 'Value');
    link_name = link_names{current_link_name};
    
    body_idx = handles.robot_model.findLinkId(link_name);
    body = handles.robot_model.getBody(body_idx);
    
    old_body_com = body.com;
    
    set(handles.current_link_com_value, 'String', num2str(old_body_com'));
    setappdata(handles.fix_link_com_ui_fig, 'old_body_com', old_body_com);
    setappdata(handles.fix_link_com_ui_fig, 'current_body_idx', body_idx);
    
    set(handles.delta_x_slider, 'Value', 0);
    set(handles.delta_y_slider, 'Value', 0);
    set(handles.delta_z_slider, 'Value', 0);
    handle_slider_change(handles);
    

% --- Executes during object creation, after setting all properties.
function current_link_combobox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to current_link_combobox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function delta_x_slider_Callback(hObject, eventdata, handles)
% hObject    handle to delta_x_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    handle_slider_change(handles);


% --- Executes during object creation, after setting all properties.
function delta_x_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_x_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function delta_z_slider_Callback(hObject, eventdata, handles)
% hObject    handle to delta_z_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    handle_slider_change(handles);
    

% --- Executes during object creation, after setting all properties.
function delta_z_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_z_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function delta_y_slider_Callback(hObject, eventdata, handles)
% hObject    handle to delta_y_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    handle_slider_change(handles);
    

% --- Executes during object creation, after setting all properties.
function delta_y_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to delta_y_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function delta_x_value_Callback(hObject, eventdata, handles)
% hObject    handle to delta_x_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_x_value as text
%        str2double(get(hObject,'String')) returns contents of delta_x_value as a double
    delta_x_str = get(hObject, 'String');
    delta_x = str2double(delta_x_str);
    
    max_value = get(handles.delta_x_slider, 'Max');
    min_value = get(handles.delta_x_slider, 'Min');
        
    if ( ~isempty(delta_x) && delta_x <= max_value && delta_x >= min_value ) % only accept valid values
        set(handles.delta_x_slider, 'Value', delta_x);
    end
    
    handle_slider_change(handles);

function delta_y_value_Callback(hObject, eventdata, handles)
% hObject    handle to delta_y_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_y_value as text
%        str2double(get(hObject,'String')) returns contents of delta_y_value as a double
    delta_y_str = get(hObject, 'String');
    delta_y = str2double(delta_y_str);
    
    max_value = get(handles.delta_y_slider, 'Max');
    min_value = get(handles.delta_y_slider, 'Min');
        
    if ( ~isempty(delta_y) && delta_y <= max_value && delta_y >= min_value ) % only accept valid values
        set(handles.delta_y_slider, 'Value', delta_y);
    end
    
    handle_slider_change(handles);



function delta_z_value_Callback(hObject, eventdata, handles)
% hObject    handle to delta_z_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of delta_z_value as text
%        str2double(get(hObject,'String')) returns contents of delta_z_value as a double
    delta_z_str = get(hObject, 'String');
    delta_z = str2double(delta_z_str);
    
    max_value = get(handles.delta_z_slider, 'Max');
    min_value = get(handles.delta_z_slider, 'Min');
        
    if ( ~isempty(delta_z) && delta_z <= max_value && delta_z >= min_value ) % only accept valid values
        set(handles.delta_z_slider, 'Value', delta_x);
    end
    
    handle_slider_change(handles);
    
    
function handle_slider_change(handles)
    delta_x = get(handles.delta_x_slider, 'Value');
    delta_y = get(handles.delta_y_slider, 'Value');
    delta_z = get(handles.delta_z_slider, 'Value');
    
    set(handles.delta_x_value, 'String', sprintf('% .3f', delta_x));
    set(handles.delta_y_value, 'String', sprintf('% .3f', delta_y))
    set(handles.delta_z_value, 'String', sprintf('% .3f', delta_z))
    
    body_idx     = getappdata(handles.fix_link_com_ui_fig, 'current_body_idx');    
    old_body_com = getappdata(handles.fix_link_com_ui_fig, 'old_body_com');
    
    new_body_com = old_body_com + [delta_x; delta_y; delta_z];
       
    body = handles.robot_model.body(body_idx);
    body = body.setInertial(body.mass, new_body_com, body.inertia);
    handles.robot_model = handles.robot_model.setBody(body_idx, body);
    handles.robot_model = handles.robot_model.compile();

    if ( ~isempty(handles.drake_ik_interface ))        
        handles.drake_ik_interface.update_robot_model(handles.robot_model);
    end
    
    robot_com = handles.robot_model.getCOM(handles.current_robot_pose);
  
    set(handles.current_link_com_value, 'String', sprintf('% .3f   ', new_body_com'));
    set(handles.current_com_value, 'String', sprintf('% .3f   ', robot_com'));
    
    cla(handles.robot_com_figure);
    hull_distance = plot_robot_com(handles.robot_model, handles.current_robot_pose, 'CoM state');
    set(handles.hull_distance_value, 'String', sprintf('% .3f', hull_distance));
    
    disp( sprintf('Done with slider change to position [% .3f, % .3f, % .3f]', delta_x, delta_y, delta_z) );
    guidata(handles.fix_link_com_ui_fig, handles);


function current_robot_pose = handle_new_joint_state(joint_state_msg, robot_model, old_robot_pose)
    message_joint_names = joint_state_msg.name;
    message_qs = joint_state_msg.position;
    
    current_robot_pose = old_robot_pose;

    for i = 1:length(message_qs)
        current_name = message_joint_names{i};
        current_q = message_qs(i);

        body_idx = robot_model.findJointId(current_name);
        model_q_idx = robot_model.getBody(body_idx).position_num;
        current_robot_pose(model_q_idx) = current_q;
    end
    
function handles = init_ros(handles)
    addpath('/usr/local/MATLAB/R2014a/ros/indigo/matlab');
    sensor_msgs;

    ros.init();
    
    % add vigir paths to ros package path (needed to load robot
    % model correctly)
    vigir_paths = getenv('ROS_VIGIR_PACKAGE_PATH');
    rosmatlab_paths = getenv('ROS_PACKAGE_PATH');
    if ( length([rosmatlab_paths ':' vigir_paths]) <= 32766 )
        setenv('ROS_PACKAGE_PATH', [rosmatlab_paths ':' vigir_paths]);
    else
        warning('Unable to set ROS_PACKAGE_PATH');
    end
    
    handles.joint_state_sub = ros.Subscriber('/thor_mang/joint_states','sensor_msgs/JointState', 1);
    handles.imu_sub = ros.Subscriber('/thor_mang/pelvis_imu', 'sensor_msgs/Imu', 1);

        
function handles = init_robot_model(handles)
    handles.robot_model = RigidBodyManipulator();

    options.floating = 'rpy';
    urdf_string = ros.param.get('/robot_description');

    w = warning('off','Drake:RigidBodyManipulator:ReplacedCylinder');
    warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
    handles.robot_model = addRobotFromURDFString(handles.robot_model,urdf_string,[],[],options);
    warning(w);

    % load nominal pose from param server
    pose_str = ros.param.get('/drake_nominal_pose');
    if ( isempty(pose_str) )
        ros.log('ERROR', 'Please specify /drake_nominal_pose ROS parameter...');
        error('Please specify /drake_nominal_pose ROS parameter...');
    end
    robot_nominal_pose = sscanf(pose_str, '%f, ');
    if ( length(robot_nominal_pose) ~= handles.robot_model.num_positions)
        ros.log('WARN', 'Nominal pose does not match number of robot joints. Setting to 0');
        robot_nominal_pose = zeros(handles.robot_model.num_positions, 1);
    end

    handles.current_robot_pose = robot_nominal_pose;



function hull_distance_value_Callback(hObject, eventdata, handles)
% hObject    handle to hull_distance_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of hull_distance_value as text
%        str2double(get(hObject,'String')) returns contents of hull_distance_value as a double


% --- Executes during object creation, after setting all properties.
function hull_distance_value_CreateFcn(hObject, eventdata, handles)
% hObject    handle to hull_distance_value (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
