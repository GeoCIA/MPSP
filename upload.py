# ===============================================================================
# Copyright 2017 ross
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ===============================================================================

# ============= enthought library imports =======================
# ============= standard library imports ========================
# ============= local library imports  ==========================
import os
import shutil

droot = '/Volumes/NO NAME/'
sroot = os.path.abspath(os.path.dirname(__file__))

for pkg in ('mavlink', 'mpsp'):
    dest = os.path.join(droot, pkg)
    src = os.path.join(sroot, pkg)

    if os.path.isdir(dest):
        print('deleting {}'.format(dest))
        shutil.rmtree(dest)

    print('copy {} to {}'.format(src, dest))

    shutil.copytree(src, dest)
#
for src in ('boot.py','main.py','mpsp_main.py','oled.py','README.md'):
    shutil.copy(src, dest)
# ============= EOF =============================================
