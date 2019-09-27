function varargout = RobotSimulation(varargin)
    % ROBOTSIMULATION MATLAB code for RobotSimulation.fig
    %      ROBOTSIMULATION, by itself, creates a new ROBOTSIMULATION or raises the existing
    %      singleton*.

    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name',       mfilename, ...
                       'gui_Singleton',  gui_Singleton, ...
                       'gui_OpeningFcn', @RobotSimulation_OpeningFcn, ...
                       'gui_OutputFcn',  @RobotSimulation_OutputFcn, ...
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

    
function stop_timer(hObject, handles)
    %disp('stop_timer(handles)');
    if strcmp(get(handles.timer, 'Running'), 'on')
        stop(handles.timer);
    end
    
    handles.stop_timer = true;
    
    % Update handles structure
    guidata(hObject, handles);
    
function start_timer(hObject, handles)
    %disp('start_timer(handles)');
    if strcmp(get(handles.timer, 'Running'), 'off') && ~handles.stop_timer
        start(handles.timer);
    end
    
    % Update handles structure
    guidata(hObject, handles);
    
function compute_path(hObject, handles)
    %disp('start compute_path')
    axes(handles.axes1);
    cla;

    %stop_timer(hObject, handles);

    %disp(handles.source)
    %disp(handles.goal)
    
    if handles.source(1) < 1, handles.source(1) = 1; end
    if handles.source(2) < 1, handles.source(2) = 1; end
    if handles.goal(1) < 1, handles.goal(1) = 1; end
    if handles.goal(2) < 1, handles.goal(2) = 1; end
    
    handles.image_data(handles.goal(1), handles.goal(2) ) = 160;
    handles.image_data(handles.source(1), handles.source(2) ) = 160;
    image(handles.image_data, 'Parent', handles.axes1);
    
    index_algorithm = get(handles.ppm_algorithm, 'Value');
    name_algorithm = ( get(handles.ppm_algorithm, 'String') );
    name_algorithm = name_algorithm{index_algorithm};
    handles.image_data = handles.image_data_original;
    guidata(hObject, handles);
    
    pathFound = false;
     tic;
    switch(index_algorithm)
        case 1
            %disp('Processing: A* algorithm');
            [final_path, pathFound] = a_start_compute_path(handles.map, handles.mapOriginal, ...
                handles.source, handles.goal, handles.display_process, ...
                handles.resolution_x, handles.resolution_y);          
        case 2
            %disp('Processing: Potential field algorithm');
            [final_path, pathFound] = potential_field_compute_path(handles.map, handles.mapOriginal, ...
                handles.source, handles.goal, handles.display_process, ...
                handles.resolution_x, handles.resolution_y); 
        case 3
            %disp('Processing: Bidirectional RRT');
            [final_path, pathFound] = bidirectional_RRT_compute_final_path(handles.map, handles.mapOriginal, ...
                handles.source, handles.goal, handles.display_process, ...
                handles.resolution_x, handles.resolution_y); 
        case 4
            %disp('Probabilistic roadmap');
            [final_path, pathFound] = probabilistic_roadmap_final_path(handles.map, handles.mapOriginal, ...
                handles.source, handles.goal, handles.display_process, ...
                handles.resolution_x, handles.resolution_y); 
    end
    
    computation_time = toc;
    lp = length(handles.final_path);  
    fprintf('Algorithm: %s - Path length: %d - Computation time: %d \n', name_algorithm, lp, computation_time);
    
    %disp(final_path)
    image(handles.image_data, 'Parent', handles.axes1);
    line(final_path(:,2),final_path(:,1));
    
    handles.final_path = final_path;
    
    handles.actual_position = handles.source;
    handles.actual_index_path = 1;

    %handles.stop_timer = false;
    
    guidata(hObject, handles); 
    
    update_robot_position(hObject, handles);  
    
    final_path_unique = unique(final_path, 'rows');
    
    if( length(final_path) < 3 || ~pathFound || length(final_path_unique) < 3)
        handles.stop_timer = true;
    end
    
    guidata(hObject, handles); 

function update_robot_position(hObject, handles)
    handles.image_data = handles.image_data_original;
    handles.image_data(handles.goal(1), handles.goal(2) ) = 160;
    handles.image_data(handles.actual_position(1), handles.actual_position(2) ) = 160;
    image(handles.image_data, 'Parent', handles.axes1);
    aip = handles.actual_index_path;
    lp = length(handles.final_path);
    line( handles.final_path((aip:lp),2), handles.final_path((aip:lp),1),...
        'Color', 'red', 'LineWidth', 3 );
    %disp(  lp )
     pause(1/1000); %pauses for 5 milliseconds.
    
     if ( handles.stop_timer )
        stop_timer(hObject, handles);
     end

function update_display(hObject, eventdata, hfigure)
    pause(1/1000);
    % Timer timer1 callback, called each time timer iterates.
    handles = guidata(hfigure);
        
    if  handles.actual_index_path < length(handles.final_path)  
        handles.actual_index_path = handles.actual_index_path + 1;
        handles.actual_position = handles.final_path(handles.actual_index_path, (1:2) );

        % Add gaussian noise
        factor_noise = (1 / 5000 ) * length(handles.final_path);

        x_max = int16(handles.resolution_x * factor_noise );
        y_max = int16(handles.resolution_y * factor_noise );

        noise_x = handles.actual_position(1) + randi([-x_max, x_max ], 1, 1) ;
        noise_y = handles.actual_position(2) + randi([-y_max, y_max ], 1, 1) ;

        if noise_x > handles.resolution_x, noise_x = handles.resolution_x; end
        if noise_y > handles.resolution_y, noise_y = handles.resolution_y; end
        
        if noise_x < 1 handles.resolution_x, noise_x = 1; end
        if noise_y < 1 handles.resolution_y, noise_y = 1; end
        
        if(handles.map(noise_x, noise_y) ~= 0 )
            handles.actual_position(1) = noise_x;
            handles.actual_position(2) = noise_y;
        end
        
        handles.source = handles.actual_position;

        guidata(hfigure, handles); 

        if(handles.map(noise_x, noise_y) ~= 0 )
            compute_path(hfigure, handles);
        else
            update_robot_position(hfigure, handles);
        end        
        
        %update_robot_position(hfigure, handles);
    else
        %stop_timer(hfigure, handles);
    end
   
% --- Executes just before RobotSimulation is made visible.
function RobotSimulation_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to RobotSimulation (see VARARGIN)

    % Choose default command line output for RobotSimulation
    handles.output = hObject;

    % This sets up the initial plot - only do when we are invisible
    % so window can get raised using RobotSimulation.
    if strcmp(get(hObject,'Visible'),'off')
        plot(0);
    end
    
    handles.resolution_x = 100;
    handles.resolution_y = 100;
    
    % Create a timer object to fire at 1/10 sec intervals
    % Specify function handles for its start and run callbacks
    handles.timer = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 0.1, ...                        % Initial period is 1 sec.
    'TimerFcn', {@update_display, hObject}); % Specify callback function

    handles.actual_index_path = 1;
    handles.final_path = 1;
    handles.stop_timer = false;

    % Update handles structure
    guidata(hObject, handles);
    
    start_timer(hObject, handles);

    % UIWAIT makes RobotSimulation wait for user response (see UIRESUME)
    uiwait(handles.figure1);

% --- Outputs from this function are returned to the command line.
function varargout = RobotSimulation_OutputFcn(hObject, eventdata, handles)
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    %varargout{1} = handles.output;

% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
    % hObject    handle to FileMenu (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
    % hObject    handle to OpenMenuItem (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    file = uigetfile('*.fig');
    if ~isequal(file, 0)
        open(file);
    end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
    % hObject    handle to PrintMenuItem (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
    % hObject    handle to CloseMenuItem (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                         ['Close ' get(handles.figure1,'Name') '...'],...
                         'Yes','No','Yes');
    if strcmp(selection,'No')
        return;
    end

    delete(handles.figure1)

% --- Executes on selection change in ppm_map.
function ppm_map_Callback(hObject, eventdata, handles)
    % hObject    handle to ppm_map (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns ppm_map contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from ppm_map


% --- Executes during object creation, after setting all properties.
function ppm_map_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to ppm_map (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

    set(hObject, 'String', {' Map 1', ' Map 2', ' Map 3', ' Map 4', ' Map 5'});

% --- Executes on selection change in ppm_algorithm.
function ppm_algorithm_Callback(hObject, eventdata, handles)
    % hObject    handle to ppm_algorithm (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: contents = cellstr(get(hObject,'String')) returns ppm_algorithm contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from ppm_algorithm

% --- Executes during object creation, after setting all properties.
function ppm_algorithm_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to ppm_algorithm (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

    set(hObject, 'String', {' A*', ' Potential field', ' Bidirectional RRT', ' Probabilistic Roadmap'});

% --- Executes on button press in btn_update.
function btn_update_Callback(hObject, eventdata, handles)
    % hObject    handle to btn_update (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    %stop_timer(hObject, handles);
    axes(handles.axes1);
    cla;

    index = get(handles.ppm_map, 'Value');
    [image_data, image_original, map, mapOriginal] = LoadMap(index, handles.resolution_x, handles.resolution_y);
    colormap(gray(256));
    image(image_data,'Parent', handles.axes1);

    [x,y] = ginput(1);
    source=[double(int32(y)) double(int32(x))];    % source position in Y, X format

    [x,y] = ginput(1);
    goal=[double(int32(y)) double(int32(x))];    % source position in Y, X format

    index = get(handles.ppm_view, 'Value');
    display_process = false;

    if(index == 2)  
        display_process = true ;
    end;

    handles.display_process = display_process;
    
    if(display_process == true)
        handles.stop_timer = true;
    end;
        
    handles.image_data = image_data;
    handles.image_data_original = image_original;
    handles.map = map;
    handles.mapOriginal = mapOriginal;
    handles.source = source;
    handles.goal = goal;
    guidata(hObject, handles)

    compute_path(hObject, handles);

% --- Executes on button press in btn_initial_position.
function btn_initial_position_Callback(hObject, eventdata, handles)
    % hObject    handle to btn_initial_position (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    %stop_timer(hObject, handles);
    
    [x,y] = ginput(1);
    source=[double(int32(y)) double(int32(x))];    % source position in Y, X format

    handles.source = source;
    guidata(hObject, handles)

    compute_path(hObject, handles);

% --- Executes on button press in btn_goal_position.
function btn_goal_position_Callback(hObject, eventdata, handles)
    % hObject    handle to btn_goal_position (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    %stop_timer(hObject, handles);
    
    [x,y] = ginput(1);
    goal=[double(int32(y)) double(int32(x))];    % source position in Y, X format

    handles.goal = goal;
    guidata(hObject, handles)

    compute_path(hObject, handles);

% --- Executes on selection change in ppm_view.
function ppm_view_Callback(hObject, eventdata, handles)
    % hObject    handle to ppm_view (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    index = get(handles.ppm_view, 'Value');
    display_process = false;

    if(index == 2)  
        display_process = true ;
    end;

    handles.display_process = display_process;
    guidata(hObject, handles)

% --- Executes during object creation, after setting all properties.
function ppm_view_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to ppm_view (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end

    set(hObject, 'String', {' Real time - online', ' Animated - off line' });

% --- Executes on button press in btn_stop.
function btn_stop_Callback(hObject, eventdata, handles)
% hObject    handle to btn_stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    %disp('stop simulation...');
    handles.stop_timer = true;
    guidata(hObject, handles)
    
    stop_timer(hObject, handles);

% --- Executes on button press in btn_random_goal.
function btn_random_goal_Callback(hObject, eventdata, handles)
% hObject    handle to btn_random_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.stop_timer = false;
random_found= false;
while(~random_found)
    i_x  = randi(handles.resolution_x/2);
    i_y  = randi(handles.resolution_y/2);
    
    if(handles.map(i_x, i_y) ~= 0 )
        handles.goal = [i_x, i_y] ;
        guidata(hObject, handles)
        
        compute_path(hObject, handles);
        
        random_found = true;
    end
end

start_timer(hObject, handles);

% --- Executes on button press in btn_random_source.
function btn_random_source_Callback(hObject, eventdata, handles)
% hObject    handle to btn_random_source (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.stop_timer = false;
random_found= false;
while(~random_found)
    i_x  = randi(handles.resolution_x/2);
    i_y  = randi(handles.resolution_y/2);
    
    if(handles.map(i_x, i_y) ~= 0 )
        handles.source = [i_x, i_y] ;
        guidata(hObject, handles)
        
        compute_path(hObject, handles);
        
        random_found = true;
    end
end

start_timer(hObject, handles);

% --- Executes during object creation, after setting all properties.
function periodsldr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to periodsldr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes on slider movement.
function periodsldr_Callback(hObject, eventdata, handles)
% hObject    handle to periodsldr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Read the slider value
period = get(handles.periodsldr,'Value');
% Timers need the precision of periods to be greater than about
% 1 millisecond, so truncate the value returned by the slider
period = period - mod(period,.01);
% Set slider readout to show its value
set(handles.slidervalue,'String',num2str(period))
% If timer is on, stop it, reset the period, and start it again.

stop_timer(hObject, handles);
set(handles.timer,'Period',period)
handles.stop_timer = false;
guidata(hObject, handles)

start_timer(hObject, handles);
