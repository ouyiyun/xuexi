function varargout = ex_guide_timergui(varargin)
% EX_GUIDE_TIMERGUI - Execute graphic updates at regular intervals
%   MATLAB code for ex_guide_timergui.fig
%      EX_GUIDE_TIMERGUI, by itself, creates a new EX_GUIDE_TIMERGUI 
%      or raises the existing singleton*.
%
%      H = EX_GUIDE_TIMERGUI returns the handle to a new EX_GUIDE_TIMERGUI
%      or the handle to the existing singleton*.
%
%      EX_GUIDE_TIMERGUI('CALLBACK',hObject,eventData,handles,...) calls
%      the local function named CALLBACK in EX_GUIDE_TIMERGUI.M with 
%      the given input arguments.
%
%      EX_GUIDE_TIMERGUI('Property','Value',...) creates a new 
%      EX_GUIDE_TIMERGUI or raises the existing singleton*.
%      Starting from the left, property value pairs are applied to the 
%      GUI before ex_guide_timergui_OpeningFcn gets called.
%      An unrecognized property name or invalid value makes property
%      application stop.  All inputs are passed to 
%      ex_guide_timergui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows
%      only one instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES, TIMER

% Last Modified by GUIDE v2.5 12-Jan-2011 11:49:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ex_guide_timergui_OpeningFcn, ...
                   'gui_OutputFcn',  @ex_guide_timergui_OutputFcn, ...
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


% --- Executes just before ex_guide_timergui is made visible.
function ex_guide_timergui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ex_guide_timergui (see VARARGIN)

% Choose default command line output for ex_guide_timergui
handles.output = hObject;

% START USER CODE
% Create a timer object to fire at 1/10 sec intervals
% Specify function handles for its start and run callbacks
handles.timer = timer(...
    'ExecutionMode', 'fixedRate', ...       % Run timer repeatedly
    'Period', 1, ...                        % Initial period is 1 sec.
    'TimerFcn', {@update_display,hObject}); % Specify callback function
% Initialize slider and its readout text field
set(handles.periodsldr,'Min',0.01,'Max',2)
set(handles.periodsldr,'Value',get(handles.timer,'Period'))
set(handles.slidervalue,'String',...
    num2str(get(handles.periodsldr,'Value')))
% Create a surface plot of peaks data. Store handle to it.
handles.surf = surf(handles.display,peaks);
% END USER CODE

% Update handles structure
guidata(hObject,handles);


% --- Outputs from this function are returned to the command line.
function varargout = ex_guide_timergui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in startbtn.
function startbtn_Callback(hObject, eventdata, handles)
% hObject    handle to startbtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% START USER CODE
% Only start timer if it is not running
if strcmp(get(handles.timer, 'Running'), 'off')
    start(handles.timer);
end
% END USER CODE


% --- Executes on button press in stopbtn.
function stopbtn_Callback(hObject, eventdata, handles)
% hObject    handle to stopbtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% START USER CODE
% Only stop timer if it is running
if strcmp(get(handles.timer, 'Running'), 'on')
    stop(handles.timer);
end
% END USER CODE


% --- Executes on slider movement.
function periodsldr_Callback(hObject, eventdata, handles)
% hObject    handle to periodsldr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% START USER CODE
% Read the slider value
period = get(handles.periodsldr,'Value');
% Timers need the precision of periods to be greater than about
% 1 millisecond, so truncate the value returned by the slider
period = period - mod(period,.01);
% Set slider readout to show its value
set(handles.slidervalue,'String',num2str(period))
% If timer is on, stop it, reset the period, and start it again.
if strcmp(get(handles.timer, 'Running'), 'on')
    stop(handles.timer);
    set(handles.timer,'Period',period)
    start(handles.timer)
else               % If timer is stopped, reset its period only.
    set(handles.timer,'Period',period)
end
% END USER CODE


% --- Executes during object creation, after setting all properties.
function periodsldr_CreateFcn(hObject, eventdata,handles)
% hObject    handle to periodsldr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(groot,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% START USER CODE
function update_display(hObject,eventdata,hfigure)
% Timer timer1 callback, called each time timer iterates.
% Gets surface Z data, adds noise, and writes it back to surface object.

handles = guidata(hfigure);
Z = get(handles.surf,'ZData');
Z = Z + 0.1*randn(size(Z));
set(handles.surf,'ZData',Z);
% END USER CODE


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% START USER CODE
% Necessary to provide this function to prevent timer callback
% from causing an error after GUI code stops executing.
% Before exiting, if the timer is running, stop it.
if strcmp(get(handles.timer, 'Running'), 'on')
    stop(handles.timer);
end
% Destroy timer
delete(handles.timer)
% END USER CODE

% Hint: delete(hObject) closes the figure
delete(hObject);
