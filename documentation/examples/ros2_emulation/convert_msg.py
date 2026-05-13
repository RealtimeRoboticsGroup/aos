#!/usr/bin/python3

# A hacky script for converting ROS2 .msg files to .fbs files. This is not
# intended to be complete or have any correctness guarantees, the output must be
# reviewed. See README.md in this folder for details.

import itertools
import os
import re
import sys

PRIMITIVE_TYPES = (
    'bool',
    'byte',
    'float32',
    'float64',
    'int8',
    'uint8',
    'int16',
    'uint16',
    'int32',
    'uint32',
    'int64',
    'uint64',
    'string',
)


def to_file_basename(msg_name):
    return re.sub(
        '[A-Z][a-z]', lambda match:
        ('_' if match.start(0) else '') + match.group(0).lower(),
        msg_name).lower()


class StdType:

    def __init__(self,
                 *,
                 ty,
                 emulation_ty,
                 emulation_fbs_base,
                 list_fields=[],
                 str_fields=[],
                 table_fields={},
                 py_defaults={}):
        self.ty = ty
        self.emulation_ty = emulation_ty
        self.emulation_fbs_base = emulation_fbs_base
        self.list_fields = list_fields
        self.py_defaults = py_defaults
        self.str_fields = str_fields
        self.table_fields = table_fields

    @property
    def emulation_target(self):
        return '@aos//documentation/examples/ros2_emulation:' + self.emulation_fbs_base

    @property
    def emulation_fbs(self):
        return 'documentation/examples/ros2_emulation/' + self.emulation_fbs_base

    @property
    def ros2_namespace(self):
        return self.ty.split('/', 1)[0]

    @property
    def table_name(self):
        return self.ty.split('/', 1)[1]

    @property
    def ros2_basename(self):
        return to_file_basename(self.table_name)


STD_TYPES = (
    StdType(
        ty='std_msgs/Header',
        emulation_ty='aos.ros2_emulation.Ros2Header',
        emulation_fbs_base='ros2_header',
        str_fields=['frame_id'],
        py_defaults={
            'stamp': 'Ros2TimeT()',
        },
    ),
    StdType(
        ty='std_msgs/String',
        emulation_ty='aos.ros2_emulation.Ros2String',
        emulation_fbs_base='ros2_string',
        str_fields=['data'],
    ),
    StdType(
        ty='std_msgs/UInt64',
        emulation_ty='aos.ros2_emulation.Ros2UInt64',
        emulation_fbs_base='ros2_u_int64',
    ),
    StdType(
        ty='std_msgs/Int32',
        emulation_ty='aos.ros2_emulation.Ros2Int32',
        emulation_fbs_base='ros2_int32',
    ),
    StdType(
        ty='std_msgs/Float32',
        emulation_ty='aos.ros2_emulation.Ros2Float32',
        emulation_fbs_base='ros2_float32',
    ),
    StdType(
        ty='std_msgs/Bool',
        emulation_ty='aos.ros2_emulation.Ros2Bool',
        emulation_fbs_base='ros2_bool',
    ),
    StdType(
        ty='geometry_msgs/Vector3',
        emulation_ty='aos.ros2_emulation.Ros2Vector3',
        emulation_fbs_base='ros2_geometry_vector3',
    ),
    StdType(
        ty='geometry_msgs/Quaternion',
        emulation_ty='aos.ros2_emulation.Ros2Quaternion',
        emulation_fbs_base='ros2_geometry_quaternion',
    ),
    StdType(
        ty='geometry_msgs/Point',
        emulation_ty='aos.ros2_emulation.Ros2Point',
        emulation_fbs_base='ros2_geometry_point',
    ),
    StdType(
        ty='geometry_msgs/Pose',
        emulation_ty='aos.ros2_emulation.Ros2Pose',
        emulation_fbs_base='ros2_geometry_pose',
        py_defaults={
            'position': 'Point()',
            'orientation': 'Quaternion()',
        },
    ),
    StdType(
        ty='sensor_msgs/Joy',
        emulation_ty='aos.ros2_emulation.Ros2Joy',
        emulation_fbs_base='ros2_sensor_joy',
        list_fields=['axes', 'buttons'],
        py_defaults={
            'header': 'Header()',
        },
        table_fields={
            'header': 'Header',
        },
    ),
)

STD_ROS2_NAMESPACES = set((std_type.ros2_namespace for std_type in STD_TYPES))


class MsgBuild:

    def __init__(self, project_name, msg_folder):
        self.fbs_list = list()
        self.project_name = project_name
        self.msg_folder = msg_folder

    def add_fbs(self, convert_fbs):
        self.fbs_list.append(convert_fbs)

    def write_build(self):
        self.write_import_helpers(self.project_name + '.msg',
                                  self.project_name)

        with open(os.path.join(self.msg_folder, 'BUILD'), 'w') as build:
            build.write(
                'load("@aos//aos/flatbuffers:flatbuffers_python.bzl", "flatbuffer_py_library")\n'
            )
            build.write(
                'load("@aos//aos/flatbuffers:generate.bzl", "static_flatbuffer")\n'
            )
            build.write(
                'load("@rules_python//python:defs.bzl", "py_library")\n')
            build.write(
                'load("@aos//aos:config.bzl", "aos_config_flatbuffers")\n')
            build.write('\n')
            build.write(
                'package(default_visibility = ["//visibility:public"])\n')
            build.write('\n')
            build.write(
                'filegroup(name = "msg_files", srcs = glob(["*.msg"]))\n')

            self.fbs_list.sort(key=lambda f: f.table_name)

            for fbs in self.fbs_list:
                build.write('\n')
                build.write('flatbuffer_py_library(\n')
                build.write('    name = "%s_fbs_py",\n' % fbs.file_basename)
                build.write('    src = "%s.fbs",\n' % fbs.file_basename)
                if fbs.dep_labels:
                    build.write('    deps = [\n')
                    for dep in sorted(fbs.dep_labels):
                        build.write('        "%s_fbs_py_srcs",\n' % dep)
                    build.write('    ],\n')
                build.write(')\n')

                build.write('\n')
                build.write('static_flatbuffer(\n')
                build.write('    name = "%s_fbs",\n' % fbs.file_basename)
                build.write('    srcs = ["%s.fbs"],\n' % fbs.file_basename)
                if fbs.dep_labels:
                    build.write('    deps = [\n')
                    for dep in sorted(fbs.dep_labels):
                        build.write('        "%s_fbs",\n' % dep)
                    build.write('    ],\n')
                build.write(')\n')

            build.write('\n')
            build.write('py_library(\n')
            build.write('    name = "msg_py",\n')
            build.write('    deps = [\n')
            for fbs in self.fbs_list:
                build.write('        ":%s_fbs_py",\n' % fbs.file_basename)
            for std_type in STD_TYPES:
                build.write('        "%s_fbs_py",\n' %
                            std_type.emulation_target)
            build.write('    ],\n')
            build.write('    srcs = [\n')
            build.write('        "py_import_dir/%s/msg/__init__.py",\n' %
                        self.project_name)
            for namespace in sorted(STD_ROS2_NAMESPACES):
                build.write('        "py_import_dir/%s/msg/__init__.py",\n' %
                            namespace)
            build.write('    ],\n')
            build.write('    imports = ["py_import_dir"],\n')
            build.write(')\n')

            build.write('\n')
            build.write('cc_library(\n')
            build.write('    name = "msg",\n')
            build.write('    hdrs = [\n')
            for fbs in self.fbs_list:
                build.write('        "cc_includes_dir/%s/msg/%s.hpp",\n' %
                            (self.project_name, fbs.file_basename))
            for std_type in STD_TYPES:
                build.write('        "cc_includes_dir/%s/msg/%s.hpp",\n' %
                            (std_type.ros2_namespace, std_type.ros2_basename))
            build.write('    ],\n')
            build.write('    deps = [\n')
            for fbs in self.fbs_list:
                build.write('        ":%s_fbs",\n' % fbs.file_basename)
            for std_type in STD_TYPES:
                build.write('        "%s_fbs",\n' % std_type.emulation_target)
            build.write('    ],\n')
            build.write('    includes = ["cc_includes_dir"],\n')
            build.write(')\n')

            build.write('\n')
            build.write('aos_config_flatbuffers(\n')
            build.write('    name = "all_fbs",\n')
            build.write('    flatbuffers = [\n')
            for fbs in self.fbs_list:
                build.write('        ":%s_fbs",\n' % fbs.file_basename)
            for std_type in STD_TYPES:
                build.write('        "%s_fbs",\n' % std_type.emulation_target)
            build.write('    ],\n')
            build.write(')\n')

    def write_import_helpers(self, ros2_package, fbs_path):
        py_import_folder = os.path.join(self.msg_folder, 'py_import_dir')
        import_msg_folder = os.path.join(py_import_folder,
                                         ros2_package.replace('.', '/'))
        os.makedirs(import_msg_folder, exist_ok=True)
        with open(os.path.join(import_msg_folder, '__init__.py'),
                  'w') as py_import:
            py_import.write('from flatbuffers.compat import import_numpy\n')
            py_import.write('np = import_numpy()\n')
            for fbs in self.fbs_list:
                py_import.write('from %s_fbs_py.%s.%s import %sT as _%s\n' %
                                (fbs.file_basename, fbs_path, fbs.table_name,
                                 fbs.table_name, fbs.table_name))
            for py_extra_import in sorted(
                    set(
                        itertools.chain.from_iterable(
                            fbs.py_extra_imports for fbs in self.fbs_list))):
                py_import.write(py_extra_import + '\n')
            for fbs in self.fbs_list:
                self.write_py_wrapper(py_import, fbs.table_name,
                                      fbs.list_fields, fbs.py_defaults,
                                      fbs.str_fields, fbs.str_list_fields,
                                      fbs.table_fields, fbs.table_list_fields)
        for namespace in STD_ROS2_NAMESPACES:
            import_folder = os.path.join(py_import_folder, namespace, 'msg')
            os.makedirs(import_folder, exist_ok=True)
            with open(os.path.join(import_folder, '__init__.py'),
                      'w') as py_import:
                for std_type in STD_TYPES:
                    if std_type.ros2_namespace != namespace:
                        continue
                    py_import.write(
                        'from documentation.examples.ros2_emulation.%s_fbs_py.%s import Ros2%sT as _%s\n'
                        % (std_type.emulation_fbs_base, std_type.emulation_ty,
                           std_type.table_name, std_type.table_name))
                py_import.write(
                    'from flatbuffers.compat import import_numpy\n')
                py_import.write('np = import_numpy()\n')
                if namespace == 'std_msgs':
                    py_import.write(
                        'from documentation.examples.ros2_emulation.ros2_header_fbs_py.aos.ros2_emulation.Ros2Header import Ros2TimeT\n'
                    )
                else:
                    py_import.write('from std_msgs.msg import Header\n')
                for std_type in STD_TYPES:
                    if std_type.ros2_namespace != namespace:
                        continue
                    self.write_py_wrapper(py_import, std_type.table_name,
                                          std_type.list_fields,
                                          std_type.py_defaults,
                                          std_type.str_fields, [],
                                          std_type.table_fields, [])

        cc_includes_folder = os.path.join(self.msg_folder, 'cc_includes_dir')
        for fbs in self.fbs_list:
            include_folder = os.path.join(cc_includes_folder,
                                          ros2_package.replace('.', '/'))
            self.write_cc_include(include_folder, fbs.file_basename,
                                  fbs.file_basename, fbs.table_name,
                                  fbs.table_name, ros2_package,
                                  self.project_name, 'msg/', fbs.cc_defaults)
        for std_type in STD_TYPES:
            include_folder = os.path.join(cc_includes_folder,
                                          std_type.ros2_namespace, 'msg')
            self.write_cc_include(
                include_folder,
                std_type.ros2_basename,
                std_type.emulation_fbs_base,
                std_type.table_name,
                'Ros2' + std_type.table_name,
                std_type.ros2_namespace + '.msg',
                'aos::ros2_emulation',
                'documentation/examples/ros2_emulation/',
                {},
            )

    def write_cc_include(self, include_folder, file_basename,
                         ros2_file_basename, ros2_name, table_name,
                         ros2_package, namespace, include_path, cc_defaults):
        os.makedirs(include_folder, exist_ok=True)
        with open(os.path.join(include_folder, '%s.hpp' % file_basename),
                  'w') as cc_include:
            include_guard = '%s_MSG_%s_HPP' % (self.project_name.upper(),
                                               file_basename.upper())
            ros2_namespace = ros2_package.replace('.', '::')
            cc_include.write('#ifndef %s\n' % include_guard)
            cc_include.write('#define %s\n' % include_guard)
            cc_include.write('#include <memory>"\n')
            cc_include.write('#include "%s%s_generated.h"\n' %
                             (include_path, ros2_file_basename))
            cc_include.write('namespace %s {\n' % ros2_namespace)
            cc_include.write('struct %s : public ::%s::%sT {\n' %
                             (ros2_name, namespace, table_name))
            cc_include.write('  using SharedPtr = std::shared_ptr<%s>;\n' %
                             ros2_name)
            cc_include.write(
                '  using ConstSharedPtr = std::shared_ptr<const %s>;\n' %
                ros2_name)
            cc_include.write(
                '  static std::unique_ptr<::%s::%sT> MakeUnique() { return std::make_unique<::%s::%sT>(); }\n'
                % (namespace, table_name, namespace, table_name))
            cc_include.write('  %s() {\n' % ros2_name)
            for field_name, default in sorted(cc_defaults.items()):
                for line in default:
                    cc_include.write('    %s\n' % line)
            cc_include.write('  }\n')
            cc_include.write('};\n')
            cc_include.write('}  // namespace %s\n' % ros2_namespace)
            cc_include.write('#endif  // %s\n' % include_guard)

    def write_py_wrapper(self, py_import, table_name, list_fields, py_defaults,
                         str_fields, str_list_fields, table_fields,
                         table_list_fields):
        py_import.write('class %s(_%s):\n' % (table_name, table_name))
        py_import.write('    def __init__(self, **kwargs):\n')
        py_import.write('        super().__init__()\n')
        py_import.write('        for k, v in kwargs.items():\n')
        py_import.write('            setattr(self, k, v)\n')
        for field_name in sorted(set(list_fields) | py_defaults.keys()):
            py_import.write('        if self.%s is None:\n' % field_name)
            if field_name in py_defaults:
                py_import.write('            self.%s = %s\n' %
                                (field_name, py_defaults[field_name]))
            else:
                assert field_name in list_fields
                py_import.write('            self.%s = list()\n' % field_name)
        py_import.write('\n')

        # Flatbuffers hard-codes the type instead of using "cls" in its
        # implementation of this method, maybe we could push this change
        # upstream instead of having to override it here?
        py_import.write('    @classmethod\n')
        py_import.write('    def InitFromObj(cls, fbs):\n')
        py_import.write('        x = cls()\n')
        py_import.write('        x._UnPack(fbs)\n')
        py_import.write('        return x\n')

        py_import.write('    def _UnPack(self, fbs):\n')
        py_import.write('        super()._UnPack(fbs)\n')
        py_import.write('        self._BytesToStrings()\n')
        py_import.write('    def _BytesToStrings(self):\n')
        py_import.write('        pass\n')
        for field_name in sorted(str_fields):
            py_import.write(
                '        if self.%s is not None: self.%s = self.%s.decode("utf-8")\n'
                % (field_name, field_name, field_name))
        for field_name in sorted(str_list_fields):
            py_import.write(
                '        if self.%s is not None: self.%s = [s.decode("utf-8") for s in self.%s]\n'
                % (field_name, field_name, field_name))
        for field_name in sorted(table_fields):
            py_import.write(
                '        if self.%s is not None: %s._BytesToStrings(self.%s)\n'
                % (field_name, table_fields[field_name], field_name))
        for field_name in sorted(table_list_fields):
            py_import.write('        if self.%s is not None:\n' % field_name)
            py_import.write(
                '            for t in self.%s: %s._BytesToStrings(t)\n' %
                (field_name, table_list_fields[field_name]))
        py_import.write('\n')

        py_import.write('%s.__name__ = "%sT"\n' % (table_name, table_name))
        py_import.write('%s.__module__ = _%s.__module__\n' %
                        (table_name, table_name))
        py_import.write('\n')


FIELD_MATCH = re.compile(
    '(?P<type>[a-zA-Z0-9_/]+)(?:\\[(?P<size_le><=)?(?P<array>[0-9]*)\\])? '
    '(?P<name>[a-zA-Z0-9_]*)(?: (?P<default>[][\\-a-z0-9., \'"]*))?')
ARRAY_INIT_MATCH = re.compile('\\[(.*)\\]')


class ConvertFbs:

    def __init__(self, entry):
        self.entry = entry
        self.have_static_length = False
        self.table_name = None
        self.file_basename = None
        self.dep_labels = set()
        self.dep_filenames = set()
        self.list_fields = list()
        self.py_defaults = dict()
        self.cc_defaults = dict()
        self.py_extra_imports = set()
        self.str_fields = set()
        self.str_list_fields = set()
        self.table_fields = dict()
        self.table_list_fields = dict()

    def add_dep(self, is_final, label, filename):
        if is_final:
            assert label in self.dep_labels
            assert filename in self.dep_filenames
        else:
            self.dep_labels.add(label)
            self.dep_filenames.add(filename)

    def convert_field(self, msg_field, fbs_id, is_final):  # noqa: C901
        parsed = FIELD_MATCH.fullmatch(msg_field)
        if not parsed:
            raise RuntimeError('Failed to parse msg field: %s' % msg_field)

        field_name = parsed.group('name')
        result = [field_name, ':']

        array_match = parsed.group('array')
        if array_match is not None:
            self.list_fields.append(field_name)
            result.append('[')

        ty = parsed.group('type')

        if ty in PRIMITIVE_TYPES:
            result.append(ty)

            if ty == 'string':
                if array_match is None:
                    self.str_fields.add(field_name)
                else:
                    self.str_list_fields.add(field_name)
        else:
            if std_type := next(filter(lambda t: t.ty == ty, STD_TYPES), None):
                result.append(std_type.emulation_ty)
                self.add_dep(is_final, std_type.emulation_target,
                             std_type.emulation_fbs)
                field_type = std_type.table_name
            else:
                split = ty.split('/')
                field_type = split[-1]
                if re.match('^[A-Z]', split[-1]):
                    first = True
                    for s in split:
                        if first:
                            first = False
                        else:
                            result.append('.')
                        basename = to_file_basename(s)
                        self.add_dep(is_final, ':' + basename,
                                     'msg/' + basename)
                        result.append(s)
                else:
                    raise RuntimeError('Not sure how to handle msg type: %s' %
                                       ty)

            if array_match is None:
                self.table_fields[field_name] = field_type
            else:
                self.table_list_fields[field_name] = field_type

        if array_match is not None:
            result.append(']')

        if ty not in PRIMITIVE_TYPES:
            if '/' in ty:
                split = ty.rsplit('/', 1)
                self.py_extra_imports.add('from %s.msg import %s' %
                                          (split[0], split[1]))

        default_match = parsed.group('default')
        if ty == 'string':
            if default_match is not None and default_match not in ('""', "''"):
                self.py_defaults[field_name] = default_match
                if default_match.startswith("'"):
                    assert default_match.endswith("'")
                elif default_match.startswith('"'):
                    assert default_match.endswith('"')
                else:
                    raise RuntimeError(
                        'Not sure how to parse default %r for %s.%s' %
                        (default_match, self.table_name, field_name))
                cc_default = default_match[1:-1]
                if '"' in cc_default:
                    raise RuntimeError(
                        'C++ quoting not implemented for default %r for %s.%s'
                        % (default_match, self.table_name, field_name))
                self.cc_defaults[field_name] = ('%s = "%s";' %
                                                (field_name, cc_default), )
        elif array_match is not None:
            if default_match is not None or not array_match or parsed.group(
                    'size_le') is not None:
                if default_match is not None:
                    self.py_defaults[field_name] = default_match.replace(
                        'false', 'False')
                    if array_init := ARRAY_INIT_MATCH.fullmatch(default_match):
                        cc_default = '{%s}' % array_init.group(1)
                    else:
                        cc_default = default_match
                    self.cc_defaults[field_name] = ('%s = %s;' %
                                                    (field_name, cc_default), )
            else:
                if 'int' in ty:
                    py_default_value = cc_default_value = 0
                elif 'float' in ty:
                    py_default_value = cc_default_value = 0.0
                elif ty == 'bool':
                    py_default_value = False
                    cc_default_value = 'false'
                else:
                    print(
                        'Warning: assuming None default for type %r in %s.%s' %
                        (ty, self.table_name, field_name),
                        file=sys.stderr)
                    py_default_value = None
                    cc_default_value = 'nullptr'
                py_default_str = str([py_default_value] * int(array_match))
                self.py_defaults[field_name] = py_default_str
                cc_default_str = '{' + ', '.join(
                    [str(cc_default_value)] * int(array_match)) + '}'
                self.cc_defaults[field_name] = ('%s = %s;' %
                                                (field_name, cc_default_str), )
        elif default_match is not None:
            result.append(' = ')
            result.append(default_match)
        elif ty not in PRIMITIVE_TYPES:
            self.py_defaults[field_name] = '%s()' % split[-1]

        result.append(' (id: ')
        result.append(str(fbs_id))
        if array_match or ty == 'string':
            result.append(', static_length: ')
            if array_match:
                result.append(array_match)
            else:
                result.append('100')
            if is_final:
                assert self.have_static_length
            else:
                self.have_static_length = True
        result.append(')')

        result.append(';')
        return ''.join(result)

    def convert_msg(self, project_name, is_final):  # noqa: C901
        self.table_name = self.entry.name.replace('.msg', '')
        self.file_basename = to_file_basename(
            self.entry.name.replace('.msg', ''))
        next_id = 0
        if is_final:
            out_filename = os.path.join(os.path.dirname(self.entry.path),
                                        self.file_basename + '.fbs')
        else:
            out_filename = os.devnull
        with open(self.entry.path, 'r') as msg, open(out_filename, 'w') as fbs:
            if self.dep_filenames:
                for dep in sorted(self.dep_filenames):
                    fbs.write('include "%s.fbs";\n' % dep)
                fbs.write('\n')
            fbs.write('namespace %s;\n\n' % project_name)
            if self.have_static_length:
                fbs.write('attribute "static_length";\n\n')

            initial_comment = list()
            eat_newline = False
            for line in msg:
                if initial_comment is not None:
                    if line.startswith('#'):
                        initial_comment.append(line)
                        continue
                    else:
                        if line == '\n':
                            for initial_line in initial_comment:
                                fbs.write(initial_line.replace('#', '//', 1))
                        fbs.write('table %s {\n' % self.table_name)
                        if line != '\n':
                            for initial_line in initial_comment:
                                fbs.write(initial_line.replace('#', '  //', 1))

                        eat_newline = True
                        initial_comment = None

                if line.startswith('#'):
                    fbs.write(line.replace('#', '  //', 1))
                    continue
                if line == '\n':
                    if eat_newline:
                        eat_newline = False
                    else:
                        fbs.write('\n')
                    continue
                eat_newline = False

                field_and_comment = re.split('\\s*#', line.strip(), maxsplit=1)
                fbs.write('  ')
                fbs_field = self.convert_field(field_and_comment[0], next_id,
                                               is_final)
                fbs.write(fbs_field)
                next_id += 1
                if len(field_and_comment) > 1:
                    fbs.write(' //')
                    fbs.write(field_and_comment[1])
                fbs.write('\n')

            fbs.write('}\n\n')
            fbs.write('root_type %s;\n' % self.table_name)


def main():
    project_name = sys.argv[1]
    msg_folder = sys.argv[2]
    build = MsgBuild(project_name, msg_folder)
    with os.scandir(msg_folder) as it:
        for entry in sorted(it, key=lambda e: e.name):
            if entry.name.endswith('.msg'):
                converter = ConvertFbs(entry)
                converter.convert_msg(project_name, False)
                converter.convert_msg(project_name, True)
                build.add_fbs(converter)
    build.write_build()


if __name__ == '__main__':
    main()
