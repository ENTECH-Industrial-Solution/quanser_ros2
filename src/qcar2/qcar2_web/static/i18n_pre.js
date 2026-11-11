// QCar2 phrases that mix Thai with numbers or English words (word order differs in English),
// for robot_web_ui's i18n.js (see there). Specific ones first. Loaded before /ui/i18n.js.
(() => {
  const T = '(?![฀-๿])';
  window.RUI_EN_PRE = [
    [/ยังไม่ label/g, 'not labelled'], [/label แล้ว/g, 'labelled'],
    [/label อัตโนมัติทั้งชุด/g, 'Auto-label all'],
    [/เฉพาะ class ที่เลือก(อยู่)?/g, 'only the selected class'], [/class ที่เลือก(อยู่)?/g, 'the selected class'],
    [/ถ่าย (\d+) กล้อง/g, 'Capture $1 cameras'], [/พร้อม (\d+)\/(\d+) กล้อง/g, '$1/$2 cameras ready'],
    [/พร้อม (\d+) กล้อง/g, '$1 cameras ready'], [/กล้อง CSI (\d+) ตัว/g, '$1 CSI cameras'],
    [/ตำแหน่งกล้อง (\d+) ตัว/g, '$1 camera positions'],
    [/รอบ (\d+)\/(\d+|\?)/g, 'epoch $1/$2'], [/รอบ \(epochs\)/g, 'Epochs'],
    [/หยุดเองถ้าไม่ดีขึ้น \(รอบ\)/g, 'Early stop after (epochs)'],
    [/เริ่มใหม่จากโมเดล COCO/g, 'Fresh start from the COCO model'],
    [new RegExp('(\\d+) ภาพ' + T, 'g'), '$1 images'], [new RegExp('(\\d+) กรอบ' + T, 'g'), '$1 boxes'],
  ];
})();
